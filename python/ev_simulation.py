import tkinter as tk
from tkinter import ttk
import os
import threading
import time
import math
from PIL import Image, ImageTk
from dataclasses import dataclass, field
from typing import Optional
from collections import deque

# --- EV PARAMETERS (Executive Master Spec) ---
MASS = 1850                # kg (Target Kerb Mass)
MAX_WHEEL_TORQUE = 3600    # Nm (Tesla Model 3 RWD Wheel Torque)
MAX_POWER = 210000         # W (210 kW — motor-limited per spec, software derate of Tesla RDU)
BATTERY_CAPACITY = 75 * 3600000  # 75 kWh to Joules
EFFICIENCY = 0.88          # Powertrain efficiency (Inverter + Motor)
WHEEL_RADIUS = 0.33        # m
DT = 0.05                  # 50ms time step for physics stability

# --- RESISTANCE FACTORS ---
FRONTAL_AREA = 2.2         # m^2 (Optimized Notchback Sedan)
AIR_DENSITY = 1.225        # kg/m^3
DRAG_COEFF = 0.23          # Aero target
GRAVITY = 9.81             
ROLLING_RESIST_COEFF = 0.012 # Typical EV Low-Rolling Resistance tires

# --- THERMAL SYSTEM PARAMETERS ---
# Temperature limits (°C)
# Emergency thresholds set at -20% of Tesla Model 3 real hard limits, rounded:
#   Motor:    bearing limit 130°C  × 0.8 → 105°C
#   Inverter: SiC MOSFET limit 150°C × 0.8 → 120°C
#   Battery:  cell limit 60°C      × 0.8 →  50°C
MOTOR_TEMP_NORMAL    = (0,   90)   # Derate starts at 90C
MOTOR_TEMP_OPTIMAL   = (40,  80)
MOTOR_TEMP_EMERGENCY = (-20, 140)
BATTERY_TEMP_NORMAL  = (15,  45)   # Derate starts at 45C
BATTERY_TEMP_OPTIMAL = (20,  35)
BATTERY_TEMP_EMERGENCY = (-10, 50)
INVERTER_TEMP_NORMAL   = (0,   75)   # Derate starts at 75C
INVERTER_TEMP_OPTIMAL  = (35,  65)
INVERTER_TEMP_EMERGENCY = (-20, 120)

# Thermal constants
AMBIENT_TEMP = 25.0        # °C (default outside temperature)
# Thermal capacitance C = m * c_p  [J/°C]  — Tesla Model 3 reference
# Motor: copper windings 8kg(c_p=385) + steel lam 20kg(c_p=490) + Al housing 10kg(c_p=900) → 21,880
# Battery: 4416x 21700 cells × 70g × c_p=1000 J/kg·°C → 309,000 J/°C
# Inverter: SiC cold plate Al 3kg(c_p=900) + components 1kg(c_p=700) → 3,400
MOTOR_THERMAL_MASS    = 22000.0   # J/°C
BATTERY_THERMAL_MASS  = 309000.0  # J/°C  — full 4416-cell pack
INVERTER_THERMAL_MASS = 3400.0    # J/°C

# Heat dissipation: Newton cooling coefficient h [W/°C]
# Q_cool = h * (T_component - T_ambient)
NATURAL_CONVECTION = 5      # W/°C  always-on baseline (no system needed)
PASSIVE_COOLING = 15        # W/°C  passive fins/vents
ACTIVE_FAN_COOLING = 60     # W/°C  forced air
LIQUID_COOLING_ACTIVE = 200 # W/°C  heat transfer: component → coolant
LIQUID_COOLING_PASSIVE = 80 # W/°C  liquid cooling (passive/warm)

# Ram air: h_eff = h_base + RAM_AIR_K * v²
# Radiator frontal area ~0.06m², forced convection h ∝ v^0.8 empirically
# At 100 km/h (27.8 m/s): boost ≈ 0.06 * 27.8² ≈ 46 W/°C — total ~80-120 W/°C realistic
RAM_AIR_K = 0.06  # W/°C·(m/s)²  — corrected from 0.8 (was 10x too high)
# Powertrain loop (motor + inverter): ~9L 50/50 glycol, m=9.6kg, c_p=3500 → 33,700 J/°C
COOLANT_PT_THERMAL_MASS = 33700.0   # J/°C
COOLANT_PT_RADIATOR_H   = 80.0      # W/°C  base radiator (ram air adds on top)
# Battery loop: ~6L glycol, m=6.4kg, c_p=3500 → 22,400 J/°C, chiller-assisted
COOLANT_BAT_THERMAL_MASS = 22400.0  # J/°C
COOLANT_BAT_RADIATOR_H   = 120.0    # W/°C  chiller-assisted, higher base than PT

# Regenerative braking
REGEN_MAX_TORQUE   = 1800.0  # Nm at wheels — ~50% of drive torque, typical EV regen limit
REGEN_EFFICIENCY   = 0.85    # round-trip: kinetic -> electrical -> battery
MECH_BRAKE_DECEL   = 8.0     # m/s² max mechanical brake deceleration (full pedal past 0.5)
# Losses = P_elec * (1 - η_motor)  [W as heat]
MOTOR_ETA_BASE = 0.97       # Tesla M3 PMSM peak efficiency ~97% at optimal point
MOTOR_K_COPPER = 0.06       # copper loss coefficient (scales with I^2 proportional to throttle^2)
MOTOR_K_IRON   = 0.03       # iron/eddy loss (scales with RPM)
# floor set to 0.85 — real PMSM worst case, not 0.70 (that would be ICE territory)
# Inverter: IGBT switching losses scale with current (proportional to throttle) + fixed conduction loss
# Inverter: IGBT switching losses scale with current (∝ throttle) + fixed conduction loss
INVERTER_ETA_BASE = 0.98
INVERTER_K_SWITCH = 0.03    # switching loss coefficient

# Battery: Joule heating Q = I²·R_int
BATTERY_R_INTERNAL = 0.042  # Ω  Tesla M3 75kWh: 96s×46p, cell IR≈20mΩ → 96*(20mΩ/46)=0.042Ω
BATTERY_V_NOMINAL = 400.0   # V  nominal pack voltage

THERMAL_DERATE_FACTOR = 0.70  # max torque multiplier when any device exceeds normal_max
COOLING_POLL_INTERVAL = 5.0   # seconds between cooling state updates
class CoolingAction:
    LIQUID_WARM = -1
    NONE = 0
    PASSIVE = 1
    ACTIVE_FAN = 2
    LIQUID_COLD = 3

class ThermalDevice:
    def __init__(self, name, temp_normal, temp_optimal, temp_emergency, thermal_mass):
        self.name = name
        self.temp_normal_min,    self.temp_normal_max    = temp_normal
        self.temp_optimal_min,   self.temp_optimal_max   = temp_optimal
        self.temp_emergency_min, self.temp_emergency_max = temp_emergency
        self.thermal_mass = thermal_mass
        self.temperature = AMBIENT_TEMP
        self.cooling_action = CoolingAction.NONE
        self.heat_to_coolant = 0.0

    def is_in_emergency(self, temp):
        return temp < self.temp_emergency_min or temp > self.temp_emergency_max

    def update_temperature(self, heat_generated, ambient_temp, coolant_temp, dt):
        """Newton's law of cooling. Liquid modes cool against coolant temp, others against ambient."""
        action = self.cooling_action
        if action in (CoolingAction.LIQUID_COLD, CoolingAction.LIQUID_WARM):
            h_liquid = LIQUID_COOLING_ACTIVE if action == CoolingAction.LIQUID_COLD else LIQUID_COOLING_PASSIVE
            q_liquid = h_liquid * (self.temperature - coolant_temp)
            q_air    = NATURAL_CONVECTION * (self.temperature - ambient_temp)
            q_cool   = q_liquid + q_air
            self.heat_to_coolant = max(0.0, q_liquid)
        else:
            h_air  = NATURAL_CONVECTION + {CoolingAction.PASSIVE:    PASSIVE_COOLING,
                                            CoolingAction.ACTIVE_FAN: ACTIVE_FAN_COOLING}.get(action, 0)
            q_cool = h_air * (self.temperature - ambient_temp)
            self.heat_to_coolant = 0.0
        self.temperature += ((heat_generated - q_cool) * dt) / self.thermal_mass

    def choose_cooling_action(self, speed_kmh):
        """4-zone state machine based on optimal/normal/emergency bands:
           T <= optimal_max              -> NONE   (sweet spot, no cooling needed)
           optimal_max < T <= normal_max -> PASSIVE (fins; ram air helps at speed)
           normal_max  < T              -> escalate FAN -> LIQUID_COLD
           T > emergency_max            -> emergency flag
        """
        T = self.temperature
        if T <= self.temp_optimal_max:
            # Step cooling down — already at or below ideal ceiling
            self.cooling_action = max(CoolingAction.NONE, self.cooling_action - 1)
        elif T <= self.temp_normal_max:
            # Above optimal but within normal — passive only
            # At speed, ram air through the radiator/fins does the job, otherwise natural convection
            self.cooling_action = CoolingAction.PASSIVE
        else:
            # Above normal max — escalate one step per tick
            self.cooling_action = min(self.cooling_action + 1, CoolingAction.LIQUID_COLD)
        return self.is_in_emergency(T)

class CoolantLoop:
    """Independent coolant circuit with its own thermal mass and radiator."""
    def __init__(self, ambient_temp, thermal_mass, radiator_h):
        self.temperature = ambient_temp
        self.thermal_mass = thermal_mass
        self.radiator_h = radiator_h

    def update(self, heat_absorbed, ambient_temp, speed_ms, dt): # Updates coolant temperature
        # Ram air boosts radiator dissipation proportional to dynamic pressure (v²)
        h_eff = self.radiator_h + RAM_AIR_K * speed_ms**2
        q_rad = h_eff * (self.temperature - ambient_temp)
        self.temperature += ((heat_absorbed - q_rad) * dt) / self.thermal_mass

class EVSimulation:
    def __init__(self):
        self.speed = 0.0  # m/s
        self.battery_energy = BATTERY_CAPACITY
        self.throttle  = 0.0
        self.brake     = 0.0
        self.gradient  = 0.0  # road gradient in degrees, positive = uphill
        self.distance = 0.0 # meters
        self.trip_time = 0.0 # seconds

        # Thermal system
        self.motor_thermal    = ThermalDevice("Motor",    MOTOR_TEMP_NORMAL,    MOTOR_TEMP_OPTIMAL,    MOTOR_TEMP_EMERGENCY,    MOTOR_THERMAL_MASS)
        self.battery_thermal  = ThermalDevice("Battery",  BATTERY_TEMP_NORMAL,  BATTERY_TEMP_OPTIMAL,  BATTERY_TEMP_EMERGENCY,  BATTERY_THERMAL_MASS)
        self.inverter_thermal = ThermalDevice("Inverter", INVERTER_TEMP_NORMAL, INVERTER_TEMP_OPTIMAL, INVERTER_TEMP_EMERGENCY, INVERTER_THERMAL_MASS)
        self.coolant_pt  = CoolantLoop(AMBIENT_TEMP, COOLANT_PT_THERMAL_MASS,  COOLANT_PT_RADIATOR_H)
        self.coolant_bat = CoolantLoop(AMBIENT_TEMP, COOLANT_BAT_THERMAL_MASS, COOLANT_BAT_RADIATOR_H)
        self.ambient_temp = AMBIENT_TEMP
        self.emergency_shutdown = False # Flag for critical system failure
        self.thermal_derate = False  # True when any device above normal_max
        self.smooth_limit = 1.0      # Slew-rate limited multiplier (0.0 to 1.0)
        self._cooling_poll_ticks = 0
        self._cooling_poll_every = int(COOLING_POLL_INTERVAL / DT)
        self.last_power_kw = 0.0
        self._efficiency_window = int(30.0 / DT)  # 30s rolling average for kWh/100km
        
        self.print_parameters()

    def print_parameters(self):
        print("EV Simulation Parameters:")
        print(f"  MASS: {MASS} kg")
        print(f"  MAX_WHEEL_TORQUE: {MAX_WHEEL_TORQUE} Nm")
        print(f"  MAX_POWER: {MAX_POWER} W")
        print(f"  BATTERY_CAPACITY: {BATTERY_CAPACITY / 3600000:.2f} kWh")
        print(f"  EFFICIENCY: {EFFICIENCY:.2f}")
        print(f"  WHEEL_RADIUS: {WHEEL_RADIUS:.2f} m")
        print(f"  DT: {DT:.3f} s")
        print("Resistance Factors:")
        print(f"  FRONTAL_AREA: {FRONTAL_AREA} m^2")
        print(f"  AIR_DENSITY: {AIR_DENSITY} kg/m^3")
        print(f"  DRAG_COEFF: {DRAG_COEFF:.2f}")
        print(f"  GRAVITY: {GRAVITY:.2f} m/s^2")
        print(f"  ROLLING_RESIST_COEFF: {ROLLING_RESIST_COEFF:.4f}")
        print()

    def get_parameters(self):
        return [
            ("MASS", f"{MASS} kg"),
            ("MAX_WHEEL_TORQUE", f"{MAX_WHEEL_TORQUE} Nm"),
            ("MAX_POWER", f"{MAX_POWER} W"),
            ("BATTERY_CAPACITY", f"{BATTERY_CAPACITY / 3600000:.2f} kWh"),
            ("EFFICIENCY", f"{EFFICIENCY:.2f}"),
            ("WHEEL_RADIUS", f"{WHEEL_RADIUS:.2f} m"),
            ("DT", f"{DT:.3f} s"),
            ("DRAG_COEFF", f"{DRAG_COEFF:.2f}"),
            ("ROLLING_RESIST_COEFF", f"{ROLLING_RESIST_COEFF:.4f}"),
        ]

    def update(self):
        # Calculate the Target Power Limit (Binary/Instant) based on safety flags
        """Main simulation tick."""
        target_limit = self._determine_power_limit()
        self._apply_slew_rate_limiter(target_limit)

        v = self.speed
        self.trip_time += DT
        speed_kmh = v * 3.6
        
        applied_torque = self._calculate_drive_torque(v)
        net_force = self._calculate_total_forces(v, applied_torque)

        # Physics integration
        acceleration = net_force / MASS
        self.speed = max(0.0, self.speed + acceleration * DT)

        # Energy and Thermal subsystems
        electrical_draw, regen_returned = self._process_energy_and_thermals(v, applied_torque)
        self._update_cooling_and_safety(speed_kmh)
        self._update_telemetry(v, electrical_draw, regen_returned)

        return (self.speed * 3.6, self.battery_energy / 3600000, self.distance / 1000,
                self.motor_thermal.temperature, self.battery_thermal.temperature, self.trip_time,
                self.inverter_thermal.temperature,
                self.coolant_pt.temperature, self.coolant_bat.temperature,
                self.emergency_shutdown)

    def _determine_power_limit(self) -> float:
        if self.emergency_shutdown: # If emergency, power is cut
            return 0.0
        if self.thermal_derate:
            return THERMAL_DERATE_FACTOR
        return 1.0

    def _apply_slew_rate_limiter(self, target_limit: float):
        ramp_step = 0.4 * DT
        if self.smooth_limit < target_limit: # Smoothly increase power limit
            self.smooth_limit = min(target_limit, self.smooth_limit + ramp_step)
        elif self.smooth_limit > target_limit:
            self.smooth_limit = max(target_limit, self.smooth_limit - ramp_step)

    def _calculate_drive_torque(self, v: float) -> float:
        throttle_used = self.throttle * self.smooth_limit
        angular_vel_wheel = v / WHEEL_RADIUS
        motor_rpm = (angular_vel_wheel * 60 / (2 * math.pi)) * 9.04

        if motor_rpm < 5000:
            max_avail_torque = MAX_WHEEL_TORQUE
        else: # Constant power region
            max_avail_torque = min(MAX_WHEEL_TORQUE, MAX_POWER / max(0.01, v / WHEEL_RADIUS))

        applied_torque = throttle_used * max_avail_torque
        if motor_rpm > 17800:
            rpm_limit_factor = max(0.0, (18000 - motor_rpm) / 200.0)
            applied_torque *= rpm_limit_factor
        return applied_torque

    def _calculate_total_forces(self, v: float, applied_torque: float) -> float:
        drag_force = 0.5 * DRAG_COEFF * FRONTAL_AREA * AIR_DENSITY * v**2
        rolling_force = ROLLING_RESIST_COEFF * MASS * GRAVITY
        engine_force  = applied_torque / WHEEL_RADIUS
        gradient_force = MASS * GRAVITY * math.sin(math.radians(self.gradient))
        # Braking forces
        brake_used = self.brake if not self.emergency_shutdown else 0.0
        regen_fraction = min(brake_used, 0.5) / 0.5
        mech_fraction = max(0.0, brake_used - 0.5) / 0.5
        regen_force = (regen_fraction * REGEN_MAX_TORQUE) / WHEEL_RADIUS
        mech_brake_force = mech_fraction * MECH_BRAKE_DECEL * MASS
        return engine_force - drag_force - rolling_force - regen_force - mech_brake_force - gradient_force

    def _process_energy_and_thermals(self, v: float, applied_torque: float):
        speed_kmh = v * 3.6
        dyn_eff = 0.94 if speed_kmh < 100 else max(0.80, 0.94 - (speed_kmh - 100) / 1000)
        mech_power = applied_torque * (v / WHEEL_RADIUS) # Mechanical power at the wheels
        electrical_draw = (mech_power / dyn_eff) + 500

        brake_used = self.brake if not self.emergency_shutdown else 0.0
        regen_fraction = min(brake_used, 0.5) / 0.5
        regen_force = (regen_fraction * REGEN_MAX_TORQUE) / WHEEL_RADIUS
        regen_power = regen_force * v
        regen_returned = regen_power * REGEN_EFFICIENCY

        motor_rpm = ((v / WHEEL_RADIUS) * 60 / (2 * math.pi)) * 9.04 # Motor RPM
        rpm_norm  = min(motor_rpm / 18000, 1.0)
        throttle_used = self.throttle * self.smooth_limit
        eta_inv   = max(0.88, INVERTER_ETA_BASE - INVERTER_K_SWITCH * throttle_used)
        eta_motor = max(0.85, MOTOR_ETA_BASE - MOTOR_K_COPPER * throttle_used**2 - MOTOR_K_IRON * rpm_norm)
        
        motor_heat = (mech_power + regen_power) * (1.0 - eta_motor)
        inverter_heat = (mech_power + regen_power) * (1.0 - eta_inv)
        battery_current = electrical_draw / BATTERY_V_NOMINAL
        battery_heat    = battery_current**2 * BATTERY_R_INTERNAL # Joule heating

        self.motor_thermal.update_temperature(motor_heat, self.ambient_temp, self.coolant_pt.temperature, DT)
        self.inverter_thermal.update_temperature(inverter_heat, self.ambient_temp, self.coolant_pt.temperature, DT)
        self.battery_thermal.update_temperature(battery_heat, self.ambient_temp, self.coolant_bat.temperature, DT)

        self.coolant_pt.update(self.motor_thermal.heat_to_coolant + self.inverter_thermal.heat_to_coolant, self.ambient_temp, v, DT)
        self.coolant_bat.update(self.battery_thermal.heat_to_coolant, self.ambient_temp, v, DT)
        return electrical_draw, regen_returned

    def _update_cooling_and_safety(self, speed_kmh: float):
        self._cooling_poll_ticks += 1
        if self._cooling_poll_ticks >= self._cooling_poll_every:
            self._cooling_poll_ticks = 0
            self.motor_thermal.choose_cooling_action(speed_kmh) # Update cooling actions for each device
            self.inverter_thermal.choose_cooling_action(speed_kmh)
            self.battery_thermal.choose_cooling_action(speed_kmh)

            self.emergency_shutdown = any([
                self.motor_thermal.is_in_emergency(self.motor_thermal.temperature),
                self.inverter_thermal.is_in_emergency(self.inverter_thermal.temperature),
                self.battery_thermal.is_in_emergency(self.battery_thermal.temperature)])
            # Determine if thermal derate is needed
            if self.emergency_shutdown:
                self.thermal_derate = False
            else:
                self.thermal_derate = any([
                    self.motor_thermal.temperature > MOTOR_TEMP_NORMAL[1],
                    self.inverter_thermal.temperature > INVERTER_TEMP_NORMAL[1],
                    self.battery_thermal.temperature > BATTERY_TEMP_NORMAL[1]])

    def _update_telemetry(self, v: float, electrical_draw: float, regen_returned: float):
        self.battery_energy -= electrical_draw * DT
        self.battery_energy += regen_returned * DT
        self.battery_energy = min(self.battery_energy, BATTERY_CAPACITY)
        self.last_power_kw = (electrical_draw - regen_returned) / 1000.0 # Convert to kW

        self._eff_power_sum  = getattr(self, '_eff_power_sum',  0.0) + max(0.0, self.last_power_kw)
        self._eff_speed_sum  = getattr(self, '_eff_speed_sum',  0.0) + (v * 3.6)
        self._eff_ticks      = getattr(self, '_eff_ticks',      0)   + 1
        if self._eff_ticks > self._efficiency_window:
            # drop oldest sample — approximate with decay instead of full deque for simplicity
            self._eff_power_sum *= (self._efficiency_window / (self._efficiency_window + 1))
            self._eff_speed_sum *= (self._efficiency_window / (self._efficiency_window + 1))
            self._eff_ticks      = self._efficiency_window
        
        avg_speed = self._eff_speed_sum / max(1, self._eff_ticks)
        avg_power = self._eff_power_sum / max(1, self._eff_ticks)
        self.last_efficiency_kwh100 = (avg_power / avg_speed * 100) if avg_speed > 2.0 else None # kWh/100km
        self.distance += self.speed * DT

        return (self.speed * 3.6, self.battery_energy / 3600000, self.distance / 1000,
                self.motor_thermal.temperature, self.battery_thermal.temperature,
                self.inverter_thermal.temperature,
                self.coolant_pt.temperature, self.coolant_bat.temperature,
                self.trip_time,
                self.emergency_shutdown)

    def run_physics(self, state, lock): # Physics simulation loop
        """Physics loop — runs in its own thread, writes to shared SimState."""
        while True:
            t0 = time.perf_counter()
            result = self.update()
            with lock:
                state.speed_kmh       = result[0]
                state.kwh_left        = result[1]
                state.dist_km         = result[2]
                state.motor_temp      = result[3]
                state.battery_temp    = result[4] # Original index 4
                state.trip_time       = result[5] # New index 5
                state.inverter_temp   = result[6] # Original index 5, now 6
                state.coolant_pt_temp = result[7] # Original index 6, now 7
                state.coolant_bat_temp= result[8] # Original index 7, now 8
                state.emergency       = result[9] # Original index 8, now 9
                state.thermal_derate  = self.thermal_derate
                state.power_kw        = self.last_power_kw
                state.efficiency      = self.last_efficiency_kwh100
                state.motor_action    = self.motor_thermal.cooling_action
                state.inverter_action = self.inverter_thermal.cooling_action
                state.battery_action  = self.battery_thermal.cooling_action
                state.power_history.append(self.last_power_kw)
            elapsed = time.perf_counter() - t0
            sleep = max(0.0, DT - elapsed)
            time.sleep(sleep)


@dataclass
class SimState:
    speed_kmh:        float = 0.0
    kwh_left:         float = 75.0
    dist_km:          float = 0.0
    motor_temp:       float = AMBIENT_TEMP
    battery_temp:     float = AMBIENT_TEMP
    inverter_temp:    float = AMBIENT_TEMP
    trip_time:        float = 0.0
    coolant_pt_temp:  float = AMBIENT_TEMP
    coolant_bat_temp: float = AMBIENT_TEMP
    emergency:        bool  = False
    thermal_derate:   bool  = False
    power_kw:         float = 0.0
    efficiency:       Optional[float] = None
    motor_action:     int   = 0
    inverter_action:  int   = 0
    battery_action:   int   = 0
    power_history:    deque = field(default_factory=lambda: deque([0.0] * int(60.0 / DT),
                                                                   maxlen=int(60.0 / DT)))
class EVGUI:
    def __init__(self, root):
        self.sim = EVSimulation()
        self._state = SimState()
        self._lock  = threading.Lock()
        # Start physics in background thread
        t = threading.Thread(target=self.sim.run_physics,
                             args=(self._state, self._lock), daemon=True)
        t.start()
        self.root = root
        self.root.title("EV Prototype - System Monitor")
        self.root.configure(padx=20, pady=20, bg="#000000")

        style = ttk.Style()
        style.theme_use('clam') # Base font size 11 -> 13 for better readability
        style.configure("TLabel", font=("Helvetica", 13), background="#000000", foreground="#e0e0e0")
        style.configure("TFrame", background="#000000")
        style.configure("TLabelframe", background="#000000", foreground="#00d1ff", borderwidth=1)
        style.configure("TLabelframe.Label", font=("Helvetica", 13, "bold"), background="#000000", foreground="#00d1ff")
        style.configure("Header.TLabel", font=("Verdana", 22, "bold"), background="#000000", foreground="#ffffff")
        style.configure("Dashboard.TLabel", font=("Courier", 29, "bold"), background="#000000", foreground="#00ff99")
        style.configure("Horizontal.TScale", background="#000000", troughcolor="#333333")

        header_frame = ttk.Frame(root)
        header_frame.pack(fill="x", pady=(0, 15))
        ttk.Label(header_frame, text="EV POWERTRAIN", style="Header.TLabel").pack(side="left") # Main header
        ttk.Label(header_frame, text=" v1.02 ALPHA", font=("Helvetica", 12), foreground="#666666").pack(side="left", padx=5, pady=(8, 0))

        # ── Two-column container ──────────────────────────────────────────────
        columns = ttk.Frame(root)
        columns.pack(fill="both", expand=True)

        # Configure grid for columns frame to achieve proportional sizing
        columns.grid_columnconfigure(0, weight=8) # Left column (controls & motion)
        columns.grid_columnconfigure(1, weight=9) # Mid column (10% reduction from equal share)
        columns.grid_columnconfigure(2, weight=10) # Right column (baseline)
        columns.grid_rowconfigure(0, weight=1) # Only one row

        # ── LEFT COLUMN: controls + motion ───────────────────────────────────
        left = ttk.Frame(columns)
        left.grid(row=0, column=0, sticky="nsew", padx=(10, 10))

        self.throttle_var = tk.DoubleVar(value=0.0) # Throttle control
        self.throttle_label = ttk.Label(left, text="Throttle: 0%")
        self.throttle_label.pack(anchor="w")
        throttle_slider = ttk.Scale(left, from_=0.0, to=1.0, orient="horizontal",
                  variable=self.throttle_var, command=self.on_throttle_change)
        throttle_slider.pack(fill="x")

        self.brake_var = tk.DoubleVar(value=0.0) # Brake control
        self.brake_label = ttk.Label(left, text="Brake: 0%  [regen off]")
        self.brake_label.pack(anchor="w", pady=(6, 0))
        brake_slider = ttk.Scale(left, from_=0.0, to=1.0, orient="horizontal",
                  variable=self.brake_var, command=self.on_brake_change)
        brake_slider.pack(fill="x")
        dash_frame = ttk.Frame(left) # Dashboard display frame
        dash_frame.pack(fill="x", pady=(10, 4))
        self.speed_ui = ttk.Label(dash_frame, text="0.0 km/h", style="Dashboard.TLabel")
        self.speed_ui.pack(pady=(10, 4))
        self.battery_ui = ttk.Label(left, text="Battery: 75.00 kWh", font=("Courier", 14))
        self.battery_ui.pack(anchor="w", pady=(0,2))
        self.dist_ui = ttk.Label(left, text="Distance: 0.000 km", font=("Courier", 14), foreground="#aaaaaa")
        self.dist_ui.pack(anchor="w", pady=(0,2))
        self.trip_time_ui = ttk.Label(left, text="Trip Time: 00:00:00", font=("Courier", 14), foreground="#aaaaaa")
        self.trip_time_ui.pack(anchor="w", pady=(0,2))
        
        # Power diagram (graph)
        CHART_W, CHART_H = 300, 80
        self._chart_w = CHART_W
        self._chart_h = CHART_H
        self._max_kw  = MAX_POWER / 1000.0
        
        power_frame = ttk.LabelFrame(left, text="Power (kW)", style="Param.TLabelframe")
        power_frame.pack(fill="x", pady=(8, 0), ipady=2)
        # Using tk.Label for specific border/highlight control
        self.power_label = tk.Label(power_frame, text="--- kWh/100km", font=("Courier", 13, "bold"),
                                    bg="black", fg="#00ff99", padx=8, pady=2,
                                    highlightthickness=1, highlightbackground="white")
        self.power_label.pack(anchor="w", padx=10, pady=5)
        self.power_canvas = tk.Canvas(power_frame, width=CHART_W, height=CHART_H,
                                      bg="#000000", highlightthickness=0)
        self.power_canvas.pack(padx=6, pady=(0, 6))
        self._zero_y = CHART_H // 2
        # Pre-create one line per pixel column — update coordinates each frame, no delete/redraw
        self._bars = [
            self.power_canvas.create_line(px, self._zero_y, px, self._zero_y,
                                          fill="#ff6600", width=1)
            for px in range(CHART_W)
        ]
        self.power_canvas.create_line(0, self._zero_y, CHART_W, self._zero_y,
                                      fill="#333333", width=1)

        self.emergency_ui = ttk.Label(left, text="✓ System Normal",
                                      foreground="#00ff44", font=("Helvetica", 13, "bold"))
        self.emergency_ui.pack(pady=(15, 0))

        # ── COLUMN 2: Environment & Temperatures ──────────────────────────────
        mid = ttk.Frame(columns)
        mid.grid(row=0, column=1, sticky="nsew", padx=10)

        ttk.Label(mid, text="THERMAL MONITOR", style="Header.TLabel").pack(anchor="w") # Thermal monitor header

        self.ambient_label = ttk.Label(mid, text=f"Ambient: {AMBIENT_TEMP:.1f}°C")
        self.ambient_label.pack(anchor="w", pady=(6, 0))
        self.ambient_var = tk.DoubleVar(value=AMBIENT_TEMP)
        ttk.Scale(mid, from_=-20.0, to=50.0, orient="horizontal",
                  variable=self.ambient_var, command=self.on_ambient_change).pack(fill="x")

        # ── Gradient selector: vertical swiper + PIL-rotated car image ───────────
        grad_frame = ttk.LabelFrame(mid, text="Road Gradient", style="Param.TLabelframe")
        grad_frame.pack(fill="x", pady=(8, 0)) # Road gradient control

        GCANV_W, GCANV_H = 310, 130
        self._grad_cx, self._grad_cy = GCANV_W // 2, GCANV_H // 2

        # Vertical swiper strip on the left
        SWIPE_W = 22
        self._swipe_canvas = tk.Canvas(grad_frame, width=SWIPE_W, height=GCANV_H,
                                       bg="#111", highlightthickness=1,
                                       highlightbackground="#444") # Vertical swiper for gradient
        self._swipe_canvas.pack(side="left", padx=(6, 4), pady=6)
        # Track line
        self._swipe_canvas.create_line(SWIPE_W//2, 8, SWIPE_W//2, GCANV_H-8,
                                       fill="#444", width=2)
        # Center notch
        self._swipe_canvas.create_line(4, GCANV_H//2, SWIPE_W-4, GCANV_H//2,
                                       fill="#666", width=1)
        # Thumb handle
        ty = GCANV_H // 2
        self._swipe_thumb = self._swipe_canvas.create_rectangle(
            3, ty-10, SWIPE_W-3, ty+10, fill="#2255aa", outline="#5588ff", width=1)
        self._swipe_canvas.create_text(SWIPE_W//2, 4,  anchor="n", text="▲",
                                       fill="#5588ff", font=("Helvetica", 7))
        self._swipe_canvas.create_text(SWIPE_W//2, GCANV_H-4, anchor="s", text="▼",
                                       fill="#5588ff", font=("Helvetica", 8))
        self._swipe_canvas.bind("<B1-Motion>",    self._on_grad_drag)
        self._swipe_canvas.bind("<ButtonPress-1>", self._on_grad_drag)
        self._swipe_h = GCANV_H
        self._swipe_w = SWIPE_W

        # Car image canvas (now with black background for better contrast)
        self._grad_canvas = tk.Canvas(grad_frame, width=GCANV_W, height=GCANV_H, bg="black",
                                      highlightthickness=1, highlightbackground="#333333")
        self._grad_canvas.pack(side="left", pady=6)

        # Load + crop car image with PIL
        img_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "powertrain_scheme2.png")
        self._grad_pil_base = Image.open(img_path).crop((0, 10, 1511, 695)) \
                                   .resize((GCANV_W, GCANV_H), Image.LANCZOS)
        self._grad_img_item = self._grad_canvas.create_image(
            self._grad_cx, self._grad_cy, anchor="center")
        self._grad_last_pct = 0.0
        self._update_grad_image(0.0)

        # Label for gradient percentage and description
        self.gradient_label = ttk.Label(grad_frame, text=" +0.0%\n  flat", # Font size 11 -> 13
                                        font=("Courier", 13, "bold"), width=10)
        self.gradient_label.pack(side="left", padx=8)

        # Component temps
        temps_frame = ttk.LabelFrame(mid, text="Temperatures", style="Param.TLabelframe")
        temps_frame.pack(fill="x", pady=8)
        row_bg = "#000000" # Background for temperature labels
        self.coolant_pt_ui    = ttk.Label(temps_frame, text="Coolant M/I: 25.0°C", font=("Courier", 16, "bold"), background=row_bg)
        self.motor_temp_ui    = ttk.Label(temps_frame, text="Motor:       25.0°C", font=("Courier", 16, "bold"), background=row_bg)
        self.inverter_temp_ui = ttk.Label(temps_frame, text="Inverter:    25.0°C", font=("Courier", 16, "bold"), background=row_bg)
        self.coolant_bat_ui   = ttk.Label(temps_frame, text="Coolant Bat: 25.0°C", font=("Courier", 16, "bold"), background=row_bg)
        self.battery_temp_ui  = ttk.Label(temps_frame, text="Battery:     25.0°C", font=("Courier", 16, "bold"), background=row_bg)

        # Grid layout to show logical relationships between loops and components
        self.coolant_pt_ui.grid(row=1, column=0, sticky="nw", padx=(4, 25), pady=2)
        self.motor_temp_ui.grid(row=1, column=1, sticky="w", padx=4, pady=2)
        self.inverter_temp_ui.grid(row=2, column=1, sticky="w", padx=4, pady=(0, 12))
        self.coolant_bat_ui.grid(row=3, column=0, sticky="w", padx=(4, 25), pady=2)
        self.battery_temp_ui.grid(row=3, column=1, sticky="w", padx=4, pady=2)

        # ── COLUMN 3: Unit Specifications & Actions ───────────────────────────
        right = ttk.Frame(columns)
        right.grid(row=0, column=2, sticky="nsew", padx=(10, 0))

        ttk.Label(right, text="SYSTEM INFO", style="Header.TLabel").pack(anchor="w") # System info header

        # Moved for better horizontal layout / height reduction
        params_frame = ttk.LabelFrame(right, text="Unit Specifications", style="Param.TLabelframe") # Unit specifications display
        params_frame.pack(fill="x", pady=8, ipady=4)
        for name, value in self.sim.get_parameters(): # Font size 9 -> 11
            ttk.Label(params_frame, text=f"{name}: {value}", font=("Helvetica", 11), foreground="#888888").pack(anchor="w", padx=8, pady=1)
        # Cooling actions
        cool_frame = ttk.LabelFrame(right, text="Cooling System Status", style="Param.TLabelframe")
        cool_frame.pack(fill="x", pady=4)

        self.cooling_motor_ui    = ttk.Label(cool_frame, text="Motor:    NONE")
        self.cooling_inverter_ui = ttk.Label(cool_frame, text="Inverter: NONE")
        self.cooling_battery_ui  = ttk.Label(cool_frame, text="Battery:  NONE")
        for w in (self.cooling_motor_ui, self.cooling_inverter_ui, self.cooling_battery_ui):
            w.pack(anchor="w", padx=8, pady=2)

        self.root.update_idletasks()
        # Auto-adjust window size to fit new compact layout
        self.root.geometry(f"{self.root.winfo_width()}x{self.root.winfo_height()}")
        self.run_sim()

    def on_throttle_change(self, _):
        val = self.throttle_var.get()
        self.sim.throttle = val
        self.throttle_label.config(text=f"Throttle: {val*100:.0f}%") # Update throttle label
        if val > 0.0:
            self.brake_var.set(0.0)
            self.sim.brake = 0.0
            self.brake_label.config(text="Brake: 0%  [regen off]")

    def on_brake_change(self, _):
        val = self.brake_var.get()
        self.sim.brake = val
        if val > 0.0:
            self.throttle_var.set(0.0)
            self.sim.throttle = 0.0
            self.throttle_label.config(text="Throttle: 0%") # If brake is applied, release throttle
        if val <= 0.0:
            mode = "regen off"
        elif val <= 0.5:
            pct = val / 0.5 * 100
            mode = f"regen {pct:.0f}%"
        else:
            regen_pct = 100
            mech_pct  = (val - 0.5) / 0.5 * 100
            mode = f"regen 100% + disk {mech_pct:.0f}%"
        self.brake_label.config(text=f"Brake: {val*100:.0f}%  [{mode}]") # Update brake label

    def _update_grad_image(self, pct):
        """Rotate PIL image by gradient angle on white background and update canvas."""
        angle_deg = math.degrees(math.atan(pct / 100.0))
        # Paste car onto black background before rotating so corners are black
        bg = Image.new("RGB", self._grad_pil_base.size, (0, 0, 0))
        bg.paste(self._grad_pil_base, (0, 0))
        rotated = bg.rotate(-angle_deg, resample=Image.BICUBIC, expand=False, # Rotate image
                            fillcolor=(0, 0, 0))
        self._grad_tk_img = ImageTk.PhotoImage(rotated)
        self._grad_canvas.itemconfig(self._grad_img_item, image=self._grad_tk_img)

    def _on_grad_drag(self, event):
        """Map vertical swiper position to gradient %: center=0, top=+20, bottom=-20."""
        half = self._swipe_h // 2
        pct = -(event.y - half) / half * 20.0   # up = positive = uphill
        pct = max(-20.0, min(20.0, pct))
        # Move thumb
        ty = max(10, min(self._swipe_h - 10, event.y))
        self._swipe_canvas.coords(self._swipe_thumb, 3, ty-10, self._swipe_w-3, ty+10) # Update thumb position
        self._apply_gradient(pct)

    def on_gradient_change(self, _):
        self._apply_gradient(self._grad_last_pct)

    def _apply_gradient(self, pct):
        self._grad_last_pct = pct
        angle_deg = math.degrees(math.atan(pct / 100.0))
        self.sim.gradient = angle_deg
        self._update_grad_image(pct)
        if pct > 0.1:    desc = "uphill"
        elif pct < -0.1: desc = "downhill"
        else:            desc = "flat"
        self.gradient_label.config(text=f" {pct:+5.1f}%\n  {desc}") # Update gradient label

    def on_ambient_change(self, _):
        ambient = self.ambient_var.get()
        self.sim.ambient_temp = ambient
        self.ambient_label.config(text=f"Ambient: {ambient:.1f}°C")

    def run_sim(self):
        with self._lock:
            s = self._state
            speed_kmh      = s.speed_kmh
            kwh_left       = s.kwh_left
            dist_km        = s.dist_km
            motor_temp     = s.motor_temp
            trip_time      = s.trip_time
            battery_temp   = s.battery_temp
            inverter_temp  = s.inverter_temp
            coolant_pt_temp  = s.coolant_pt_temp
            coolant_bat_temp = s.coolant_bat_temp
            emergency      = s.emergency
            kw             = s.power_kw
            eff            = s.efficiency
            motor_action   = s.motor_action
            inverter_action= s.inverter_action
            battery_action = s.battery_action
            history        = list(s.power_history)

        # Format trip time
        hours = int(trip_time // 3600)
        minutes = int((trip_time % 3600) // 60)
        seconds = int(trip_time % 60)

        self.speed_ui.config(text=f"{speed_kmh:6.1f} km/h")
        self.battery_ui.config(text=f"Battery: {kwh_left:6.2f} kWh") # Update battery display
        self.dist_ui.config(text=f"Distance: {dist_km:8.3f} km")

        eff_str = f"{eff:5.1f} kWh/100km" if eff is not None else "  --- kWh/100km"
        self.power_label.config(text=eff_str, foreground="#00ff99")

        # Update chart from snapshot history
        c = self.power_canvas
        W, H = self._chart_w, self._chart_h
        z = self._zero_y
        n = len(history)
        for px, bar in enumerate(self._bars):
            idx = min(int(px * n / W), n - 1)
            val = history[idx]
            ratio = max(-1.0, min(1.0, val / self._max_kw))
            y = z - int(ratio * z)
            c.coords(bar, px, z, px, y)
            if val < 0:
                c.itemconfig(bar, fill="#00aa44")
            elif val > self._max_kw * 0.8:
                c.itemconfig(bar, fill="#cc3300")
            else:
                c.itemconfig(bar, fill="#ff6600")
        self.trip_time_ui.config(text=f"Trip Time: {hours:02d}:{minutes:02d}:{seconds:02d}") # Update trip time display

        def temp_color(t, normal_max, emergency_max):
            if t > emergency_max:  return "red"
            if t > normal_max:     return "orange"
            return "#00ff99"

        self.coolant_pt_ui.config( text=f"Coolant M/I: {coolant_pt_temp:5.1f}°C")
        self.motor_temp_ui.config( # Update motor temperature display
            text=f"Motor:    {motor_temp:5.1f}°C",
            foreground=temp_color(motor_temp, MOTOR_TEMP_NORMAL[1], MOTOR_TEMP_EMERGENCY[1]))
        self.inverter_temp_ui.config(
            text=f"Inverter: {inverter_temp:5.1f}°C",
            foreground=temp_color(inverter_temp, INVERTER_TEMP_NORMAL[1], INVERTER_TEMP_EMERGENCY[1]))
        self.coolant_bat_ui.config(text=f"Coolant Bat: {coolant_bat_temp:5.1f}°C")
        self.battery_temp_ui.config(
            text=f"Battery:  {battery_temp:5.1f}°C",
            foreground=temp_color(battery_temp, BATTERY_TEMP_NORMAL[1], BATTERY_TEMP_EMERGENCY[1]))

        cooling_names = {-1: "LIQUID WARM", 0: "NONE", 1: "PASSIVE", 2: "ACTIVE FAN", 3: "LIQUID COLD"}
        self.cooling_motor_ui.config(
            text=f"Motor:    {cooling_names.get(motor_action, '?')}")
        self.cooling_inverter_ui.config(
            text=f"Inverter: {cooling_names.get(inverter_action, '?')}")
        self.cooling_battery_ui.config(
            text=f"Battery:  {cooling_names.get(battery_action, '?')}")

        if emergency:
            self.emergency_ui.config(text="⚠ EMERGENCY SHUTDOWN", foreground="red")
        elif s.thermal_derate:
            self.emergency_ui.config(text="⚡ THERMAL DERATE  70%", foreground="orange")
        else:
            self.emergency_ui.config(text="✓ System Normal", foreground="green")

        self.root.after(50, self.run_sim)

if __name__ == "__main__":
    root = tk.Tk()
    app = EVGUI(root)
    root.mainloop()