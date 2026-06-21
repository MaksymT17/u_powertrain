# Executive Master Specification & Engineering Blueprint: Alpha EV Prototype
**Version:** 3.0 (C++ Qt Production Core Reference)
**Classification:** Core System Architecture & Physics Specification
**Primary Reference:** C++/Qt Production Code (`/powertrain/qt/*`)

---

## 1. System Architecture & Software Topology
The Alpha EV Prototype simulation has transitioned from a Python research prototype to a production C++ implementation. The codebase follows a **Service-Oriented Architecture (SOA)**:
*   **Orchestrator (`EVPowertrainSimulator`):** Coordinates simulation ticks, manages supervisory control logic (ignition, cooling pumps, DC-DC converter hysteresis, cabin auxiliary load calculations), and holds the primary system state.
*   **Math Engine (`PowerDeliveryService`):** An isolated microservice that calculates real-time dynamics, friction limits, resistance forces, and powertrain electrical/mechanical power flows.
*   **Data Transfer Objects (DTOs):** Defined in [vehicle_dto.h](file:///Users/mba23/projects/ucar_helpers/powertrain/qt/vehicle_dto.h), these structures (`VehicleCommandDTO`, `PowerTelemetryDTO`, `ThermalTelemetryDTO`) decouple services and represent exact network interfaces for modular deployment.
*   **UI/HMI Layer (`SpecViewer` & `PowerGraph`):** A multithreaded Qt6 dashboard showcasing telemetry, dynamic car gradient tilt, wind flow visualizations, and thermal maps.

---

## 2. Vehicle Dynamics & Chassis Constraints
*   **Body Style:** Technical Notchback Sedan (3-box configuration).
*   **Structural Integrity:** Features geometric isolation with fixed rear glass and an isolated deck-lid (trunk), achieving a target torsional rigidity of **>35,000 Nm/deg**.
*   **Dimensions:**
    *   *Wheelbase:* 2,800 mm
    *   *Ground Clearance:* 170 mm
*   **Weight & Weight Distribution:**
    *   **RWD Modality:** 1,850 kg kerb mass with a **48/52** Front/Rear bias.
    *   **AWD Modality:** 1,930 kg kerb mass (adding 80 kg front propulsion assembly) with a **50/50** Front/Rear bias.

---

## 3. Propulsion & Drivetrain Modalities

### 3.1. Propulsion Unit Specifications
| Specification | Rear Propulsion Unit (RDU) | Front Propulsion Unit (FDU) |
| :--- | :--- | :--- |
| **Motor Topology** | Permanent Magnet Synchronous (PMSM) | AC Induction Motor (ACIM) |
| **Inverter Silicon** | Silicon Carbide (SiC) MOSFETs | Silicon Carbide (SiC) / IGBT |
| **Peak Wheel Torque** | 3,600 Nm | 1,200 Nm (On-Demand) |
| **Peak Regen Torque** | 1,800 Nm | 1,200 Nm |
| **Max Rotor Speed** | 18,000 RPM (Tesla RDU limit) | N/A (Freewheels without drag) |
| **Gear Reduction Ratio** | 9.04:1 (Single-speed) | 9.04:1 (Single-speed) |

### 3.2. Physics Engine Implementation Details
*   **Velocity Ceiling:** Limited by a hard rotor limit of **18,000 RPM**, capping the vehicle's physical top speed at **~247 km/h** (with a wheel radius of 0.33m/0.36m).
*   **Slew Rate Power Limiter:** Smooths sudden torque transients. The power multiplier limit (`smoothLimit`) is rate-limited to step by `0.1 * dt` (yielding a full 0.0 to 1.0 transient response in 0.5 seconds at a 50ms tick rate).
*   **Propulsion Power Budgeting:** The maximum traction power is limited by the battery nominal capacity:
    $$\text{Propulsion Limit} = \max(0.0, (P_{\text{max}} \cdot \text{smoothLimit} - P_{\text{aux\_hv}}) \cdot 0.94)$$
    *Note:* In the math engine calculations, $P_{\text{max}}$ is a hardcoded constant of **210 kW** across both RWD and AWD configurations. The UI dynamically displays 270 kW in AWD configuration to reflect combined hardware ratings, but the battery discharge limits propulsion to 210 kW.

---

## 4. Traction Control, Wheel Configurations, and Surface Coefficients

### 4.1. Wheel and Tire Specifications (`WheelConfig`)
*   **18" Eco (`ECO_235_18`):**
    *   *Dynamic Radius:* 0.33 m
    *   *Drag Coeff Multiplier:* 1.0 (Aero $C_d = 0.250$)
    *   *Rolling Resistance Coeff ($C_{rr}$):* 0.0120 (Front & Rear)
    *   *Base Grip Coefficient ($\mu$):* 0.85
*   **21" Performance Staggered (`STAGGERED_265_21`):**
    *   *Dynamic Radius:* 0.36 m (low-profile tire sizing)
    *   *Drag Coeff Multiplier:* 1.02 (Wider tires increase drag; Aero $C_d = 0.255$)
    *   *Rolling Resistance Coeff ($C_{rr}$):* 0.0080 Front (235mm) / 0.0100 Rear (265mm)
    *   *Base Grip Coefficient ($\mu$):* 0.98

### 4.2. Surface Conditions
Rolling resistance and grip are governed dynamically by the selected surface:
*   **Asphalt (0):** Baseline tire coefficients are maintained ($\mu = 0.85$ or $0.98$).
*   **Gravel (1):** $C_{rr} = 0.025$ (both axles), drag multiplier = 1.05, grip $\mu = 0.45$.
*   **Ice (2):** $C_{rr} = 0.005$ (both axles), drag multiplier = 1.0, grip $\mu = 0.15$.

### 4.3. Virtual Slip Control (VSC) and Torque Vectoring
*   **Dynamic Grip Limits:** The physics engine calculates normal force: $F_{\text{normal}} = m \cdot g \cdot \cos(\theta_{\text{gradient}})$. Peak grip torque limits are:
    $$T_{\text{grip\_rear}} = (\mu \cdot F_{\text{normal}} \cdot \text{weight\_bias}) \cdot r_{\text{wheel}}$$
    $$T_{\text{grip\_front}} = (\mu \cdot F_{\text{normal}} \cdot (1 - \text{weight\_bias})) \cdot r_{\text{wheel}}$$
*   **Traction Control (TCS):** Torque delivered to the rear axle is dynamically capped by the grip limit:
    $$T_{\text{rear}} = \min(T_{\text{target}}, T_{\text{max\_wheel\_rear}}, T_{\text{grip\_rear}})$$
*   **Dynamic Torque Vectoring (AWD):** If the rear tire grip saturates, any excess torque demand is instantly routed to the front ACIM:
    $$T_{\text{front}} = \min(T_{\text{target}} - T_{\text{rear}}, T_{\text{max\_wheel\_front}}, T_{\text{grip\_front}})$$
*   **Regenerative Braking Slip Protection:** Regenerative torque is scaled over the first 50% of brake pedal travel (from 0% to 50% pedal, peak negative torque is 1,800 Nm in RWD / 3,000 Nm in AWD). To prevent wheel lock-up, the negative regenerative torque allocated to both front and rear axles is dynamically capped by $T_{\text{grip\_front}}$ and $T_{\text{grip\_rear}}$. Past 50% pedal travel, mechanical friction brakes are applied, yielding a deceleration rate up to $8.0\text{ m/s}^2$.

---

## 5. Thermodynamics & Active Thermal Management
The vehicle relies on a split-loop liquid cooling architecture, isolating thermal regimes between powertrain components and the battery pack.

### 5.1. Component Thermal Masses & Operational Ranges
| Component | Thermal Mass ($C_{\text{thermal}}$) | Optimal Range | Normal Max | Critical Limit | Cold Limit |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Rear Motor** | 22,000 J/°C | 40°C – 80°C | 90°C | 140°C | -20°C |
| **Rear Inverter** | 3,400 J/°C | 35°C – 65°C | 75°C | 120°C | -20°C |
| **Front Motor** | 15,000 J/°C | 40°C – 80°C | 90°C | 140°C | -20°C |
| **Front Inverter** | 2,500 J/°C | 35°C – 65°C | 75°C | 120°C | -20°C |
| **Battery Pack** | 309,000 J/°C | 20°C – 35°C | 45°C | 50°C | -10°C |

### 5.2. Coolant Loops & Radiators
*   **Powertrain Loop (PT):** Glycol mixture circulating through motors and inverters.
    *   *Coolant Thermal Mass:* 33,700 J/°C
    *   *Base Radiator Heat Coefficient ($h_{\text{rad\_PT}}$):* 80.0 W/°C
*   **Battery Loop (Bat):** Dedicated glycol loop.
    *   *Coolant Thermal Mass:* 22,400 J/°C
    *   *Base Radiator Heat Coefficient ($h_{\text{rad\_Bat}}$):* 120.0 W/°C
*   **Ram Air Effect:** Radiator dissipation scales with vehicle speed: $h_{\text{ram}} = 0.06 \cdot v_{\text{rel}}^2\text{ W/°C}$.
*   **Dynamic Fan Boosts:**
    *   *PT Loop:* If any component action is `LIQUID_HIGH`, active fans provide a **+120 W/°C** boost. If any component action is `LIQUID_LOW` or `LIQUID_MED`, fans provide a **+50 W/°C** boost.
    *   *Battery Loop:* Active battery cooling provides a **+60 W/°C** fan boost.

### 5.3. Active HVAC and Heat Pumps (HV-Powered)
The orchestrator drives active high-voltage heaters and chillers under ignition:
*   **Battery Heater (3.0 kW):** Activates when battery temperature falls below **18°C**; deactivates at or above **22°C** (2°C hysteresis offset).
*   **Battery Chiller (1.0 kW):** Activates when battery temperature exceeds **37°C**; deactivates at or below **33°C** (2°C hysteresis offset).
*   **PT Loop Heater (2.0 kW):** Activates when PT coolant falls below **38°C**; deactivates at or above **82°C**.
*   **PT Loop Chiller (1.5 kW):** Activates when PT coolant exceeds **82°C**; deactivates at or below **78°C**.

### 5.4. Cooling Loop State Machine
Every **5 seconds**, the system polls component temperatures and escalates/de-escalates pump states:
*   $T \le \text{Optimal Max} \implies$ De-escalate flow rate by one step toward `TURNED_OFF` (or `LIQUID_WARM` if below optimal minimum).
*   $\text{Optimal Max} < T \le \text{Normal Max} \implies$ Pin state to `LIQUID_LOW` (or lower if already cooling).
*   $T > \text{Normal Max} \implies$ Escalate flow rate by one step: `TURNED_OFF` $\to$ `LIQUID_LOW` $\to$ `LIQUID_MED` $\to$ `LIQUID_HIGH`.
*   **Heat Dissipation Newton Coefficients ($h_{\text{liq}}$):**
    *   `TURNED_OFF` (0) / Convection: 5 W/°C (Baseline ambient air bleed)
    *   `LIQUID_WARM` (-1): 60 W/°C (Used during cold-start pre-heating)
    *   `LIQUID_LOW` (1): 120 W/°C
    *   `LIQUID_MED` (2): 250 W/°C
    *   `LIQUID_HIGH` (3): 450 W/°C

---

## 6. Safety, Pre-checks, and Limp Modes

### 6.1. Ignition Pre-checks
Before drive torque is enabled, the supervisory controller verifies that temperatures are within operational bounds:
*   **Battery Temp:** Must be between **20°C** and **35°C** (`HeatingBattery` or `CoolingBattery` active if outside).
*   **PT Coolant Temp:** Must be above **-20°C** (`HeatingPT` active if below) and below **85°C** (`CoolingPT` active if above).
*   *Outcome:* Drive state remains locked in `PrecheckStatus` and throttle is limited to 0% until pre-checks are completed.

### 6.2. Derating and Shutdown Logic
*   **Thermal Derate (70% Power):** Triggered if any component temperature exceeds its `Normal Max`.
*   **Emergency Shutdown:** Triggered if any component temperature exceeds its `Critical Limit` or falls below its `Cold Limit`.
    *   *Moving Vehicle ($v > 0.1\text{ km/h}$):* Power is limited to a **20% limp mode** for safety.
    *   *Stationary Vehicle ($v \le 0.1\text{ km/h}$):* Propulsion power is fully cut (**0% power**).
*   **Hysteresis Cooldown Lock:** To prevent rapid oscillations between normal, derate, and shutdown modes under high loads, any state transition is locked for **30 seconds** before a lower severity level can be applied.

---

## 7. 12V Auxiliary System (DC-DC Converter)
*   **12V Battery Capacity:** 60 Ah / 12V nominal, equivalent to **2.592 MJ** (2,592,000 Joules).
*   **Base Low-Voltage Load:**
    *   *System Base Load:* 250 W (VCU, ECUs, coolant pumps, diagnostic sensors)
    *   *Infotainment Load:* Configurable from 50 W to 500 W (Default: 150 W)
    *   *Lighting Load:* Low beams = 100 W; High beams = 250 W (Mutual exclusion enforced)
*   **HV to LV charging (DC-DC Converter):**
    *   *Rating:* 1.5 kW max output.
    *   *Hysteresis Controller:* Activates when 12V battery energy falls below **80%** (2.07 MJ); deactivates once it reaches **98%** (2.54 MJ).
    *   *HV Auxiliary Draw:* When active, draws up to **~1.63 kW** from the traction battery pack (accounting for a 92% conversion efficiency).
