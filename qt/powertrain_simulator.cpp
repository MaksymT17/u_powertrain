#include "powertrain_simulator.h"
#include <algorithm>
#include <spdlog/spdlog.h>
#include <QString>

EVPowertrainSimulator::EVPowertrainSimulator() {
    powerService.resetState();
    powerState = powerService.getState();
}

void EVPowertrainSimulator::setIgnition(bool on) {
    // Safety: prevent stopping the high-voltage system while the vehicle is in motion (> 1 km/h)
    if (!on && powerState.speedKmh > 1.0) { 
        spdlog::warn("Ignition stop rejected: Safety lock active while vehicle in motion ({:.1f} km/h)", powerState.speedKmh);
        return;
    }

    ignitionOn = on;
    spdlog::info("Ignition set to: {}", on ? "ON" : "OFF");

    // Reset thermal management flags when turning off
    if (!on) {
        batteryHeaterOn = false;
        batteryChillerOn = false;
        ptHeaterOn = false;
        ptChillerOn = false;
        precheckStatus = PrecheckStatus::Initializing;
    }
}

void EVPowertrainSimulator::update(double dt) {
    modeSwitchTimer += dt;

    // 1. Auxiliary Power Budgeting (12V vs High Voltage)
    // LV systems are available as soon as we "jump into the car"
    double lv_load = SYSTEM_BASE_LOAD + infotainmentPowerDraw;
    double light_power = highBeamsOn ? 250.0 : (lowBeamsOn ? 100.0 : 0.0);
    lv_load += light_power;

    double hv_aux_load = 0.0;          // Pulls from Traction Pack

    // AC Compressor is high-voltage (HV) and available in standby/ACC
    // In our single button logic, we'll allow AC to run if ignition is requested or always on
    if (acOn) {
        double temp_diff = std::abs(ambientTemp - acTargetTemp);
        hv_aux_load += std::clamp(temp_diff * 250.0, 500.0, 5000.0);
    }

    // 1a. 12V System Maintenance (DC-DC Converter)
    // If 12V battery is low, DC-DC pulls from HV Traction pack to charge it
    // Implement hysteresis: Start at 80%, Stop at 98% to prevent rapid oscillation (blinking)
    if (battery12VEnergy < (battery12VCapacity * 0.80)) {
        dcdcActive = true;
    } else if (battery12VEnergy >= (battery12VCapacity * 0.98)) {
        dcdcActive = false;
    }

    if (dcdcActive) {
        double charging_power = DCDC_MAX_POWER; 
        hv_aux_load += (charging_power / 0.92); // Accounting for 92% DC-DC efficiency
        battery12VEnergy += charging_power * dt;
    }
    battery12VEnergy -= lv_load * dt;
    battery12VEnergy = std::clamp(battery12VEnergy, 0.0, battery12VCapacity);

    // 1b. High-Voltage Thermal Management (ONLY active during Ignition)
    if (ignitionOn) {
        if (batteryHeaterOn) hv_aux_load += batteryHeaterPowerDraw;
        if (batteryChillerOn) hv_aux_load += batteryChillerPowerDraw;
        if (ptHeaterOn) hv_aux_load += ptHeaterPowerDraw;
        if (ptChillerOn) hv_aux_load += ptChillerPowerDraw;
    }

    // --- MICROSERVICE BOUNDARY: Build DTOs and call Physics Engine ---
    VehicleCommandDTO cmd;
    cmd.throttle = throttle;
    cmd.brake = brake;
    cmd.gradient = gradient;
    cmd.ignitionOn = ignitionOn;
    cmd.configuration = configuration;
    cmd.driveMode = driveMode;
    cmd.ambientTemp = ambientTemp;
    cmd.windSpeed = windSpeed;
    cmd.surfaceType = currentSurfaceType;
    cmd.hvAuxLoadW = hv_aux_load;
    cmd.isPrecheckReady = (precheckStatus == PrecheckStatus::Ready);
    cmd.wheelConfig = wheelConfig;

    ThermalTelemetryDTO thermals;
    thermals.emergencyShutdown = emergencyShutdown;
    thermals.thermalDerate = thermalDerate;
    
    bool wasMoving = powerState.speedKmh > 0.1;
    powerState = powerService.calculatePhysics(cmd, thermals, dt);

    // Reset brake to 0.0 if we just came to a full stop while driving
    if (wasMoving && powerState.speedKmh <= 0.0 && ignitionOn && precheckStatus == PrecheckStatus::Ready) {
        brake = 0.0;
    }

    // 6. Thermal Modeling
    auto calculateDissipation = [&](double temp, double refCoolantTemp, CoolingAction action, double& heatToCoolant) {
        double q_air_base = VehicleConstants::NATURAL_CONVECTION * (temp - ambientTemp);
        heatToCoolant = 0.0;

        if (action != CoolingAction::TURNED_OFF) {
            double h_liq = 0.0;
            switch(action) {
                case CoolingAction::LIQUID_WARM: 
                    h_liq = VehicleConstants::LIQUID_COOLING_WARM; 
                    // Logic: Fans remain OFF, only pump runs to distribute heat
                    break;
                case CoolingAction::LIQUID_LOW:  h_liq = VehicleConstants::LIQUID_COOLING_LOW; break;
                case CoolingAction::LIQUID_MED:  h_liq = VehicleConstants::LIQUID_COOLING_MED; break;
                case CoolingAction::LIQUID_HIGH: h_liq = VehicleConstants::LIQUID_COOLING_HIGH; break;
                default: break;
            }
            double q_liq = h_liq * (temp - refCoolantTemp);
            heatToCoolant = q_liq; // Allow bidirectional heat transfer to warm components
            return q_air_base + q_liq;
        } else {
            return q_air_base;
        }
    };

    double mToPT, iToPT, bToBat;
    double fMToPT = 0.0, fIToPT = 0.0;
    double mDiss = calculateDissipation(motorTemp, coolantPTTemp, motorAction, mToPT);
    double iDiss = calculateDissipation(inverterTemp, coolantPTTemp, inverterAction, iToPT);
    double bDiss = calculateDissipation(batteryTemp, coolantBatTemp, batteryAction, bToBat);

    if (configuration == PowertrainConfig::AWD) {
        double fmDiss = calculateDissipation(frontMotorTemp, coolantPTTemp, frontMotorAction, fMToPT);
        double fiDiss = calculateDissipation(frontInverterTemp, coolantPTTemp, frontInverterAction, fIToPT);
        frontMotorTemp += ((powerState.frontMotorHeatW - fmDiss) * dt) / VehicleConstants::FRONT_MOTOR_THERMAL_MASS;
        frontInverterTemp += ((powerState.frontInverterHeatW - fiDiss) * dt) / VehicleConstants::FRONT_INVERTER_THERMAL_MASS;
    } else {
        double fmDiss = calculateDissipation(frontMotorTemp, ambientTemp, CoolingAction::TURNED_OFF, fMToPT);
        double fiDiss = calculateDissipation(frontInverterTemp, ambientTemp, CoolingAction::TURNED_OFF, fIToPT);
        frontMotorTemp += (-fmDiss * dt) / VehicleConstants::FRONT_MOTOR_THERMAL_MASS;
        frontInverterTemp += (-fiDiss * dt) / VehicleConstants::FRONT_INVERTER_THERMAL_MASS;
    }

    motorTemp += ((powerState.rearMotorHeatW - mDiss) * dt) / VehicleConstants::MOTOR_THERMAL_MASS;
    inverterTemp += ((powerState.rearInverterHeatW - iDiss) * dt) / VehicleConstants::INVERTER_THERMAL_MASS;
    batteryTemp += ((powerState.batteryHeatW - bDiss) * dt) / VehicleConstants::BATTERY_THERMAL_MASS;

    // 7. Coolant Loop Physics (Radiator + Ram Air)
    // Radiator cooling depends on the magnitude of relative air velocity
    double v_rel = (powerState.speedKmh / 3.6) - windSpeed;
    double h_ram_boost = VehicleConstants::RAM_AIR_K * v_rel * v_rel;

    // Dynamic Fan Rejection (Hardware Logic: PT=2 fans, BAT=1 fan)
    // PT Loop: Escalate from 1 fan to 2 fans based on demand
    double pt_fan_boost = 0.0;
    if (motorAction == CoolingAction::LIQUID_HIGH || inverterAction == CoolingAction::LIQUID_HIGH || frontMotorAction == CoolingAction::LIQUID_HIGH || frontInverterAction == CoolingAction::LIQUID_HIGH)
        pt_fan_boost = 120.0; // Both fans at max
    else if (motorAction >= CoolingAction::LIQUID_LOW || inverterAction >= CoolingAction::LIQUID_LOW || frontMotorAction >= CoolingAction::LIQUID_LOW || frontInverterAction >= CoolingAction::LIQUID_LOW)
        pt_fan_boost = 50.0;  // Single fan or low-duty cycle

    // Bat Loop: Single auxiliary fan
    double bat_fan_boost = (batteryAction >= CoolingAction::LIQUID_LOW) ? 60.0 : 0.0;
    
    // Powertrain Loop (Motor + Inverter + Front unit if AWD)
    double pt_rad_diss = (VehicleConstants::COOLANT_PT_RAD_H + pt_fan_boost + h_ram_boost) * (coolantPTTemp - ambientTemp);
    coolantPTTemp += ((mToPT + iToPT + fMToPT + fIToPT - pt_rad_diss) * dt) / VehicleConstants::COOLANT_PT_MASS;

    // Battery Loop
    double heat_from_battery_heater = batteryHeaterOn ? batteryHeaterPowerDraw : 0.0; // Heater directly adds heat to coolant
    double heat_from_battery_chiller = batteryChillerOn ? -batteryChillerPowerDraw : 0.0; // Chiller removes heat
    double bat_rad_diss = (VehicleConstants::COOLANT_BAT_RAD_H + bat_fan_boost + h_ram_boost) * (coolantBatTemp - ambientTemp); // Radiator dissipates heat
    coolantBatTemp += ((bToBat + heat_from_battery_heater + heat_from_battery_chiller - bat_rad_diss) * dt) / VehicleConstants::COOLANT_BAT_MASS;

    // Powertrain Loop (Heater/Chiller)
    double heat_from_pt_heater = ptHeaterOn ? ptHeaterPowerDraw : 0.0;
    double heat_from_pt_chiller = ptChillerOn ? -ptChillerPowerDraw : 0.0;
    coolantPTTemp += ((heat_from_pt_heater + heat_from_pt_chiller) * dt) / VehicleConstants::COOLANT_PT_MASS;

    // 8. Cooling System State Machine (Polled every 5s)
    // Only run pre-checks and thermal management if Ignition is requested
    if (ignitionOn) {
        updatePrecheckLogic(); 
        updateBatteryThermalManagement();
        updatePTThermalManagement();
    }

    coolingPollTimer += dt;
    if (coolingPollTimer >= 5.0) {
        coolingPollTimer = 0.0;
        updateDeviceCooling(motorTemp, VehicleConstants::MOTOR_OPTIMAL_MIN, VehicleConstants::MOTOR_OPTIMAL_MAX, VehicleConstants::MOTOR_NORMAL_MAX, motorAction);
        updateDeviceCooling(inverterTemp, VehicleConstants::INV_OPTIMAL_MIN, VehicleConstants::INV_OPTIMAL_MAX, VehicleConstants::INV_NORMAL_MAX, inverterAction);
        updateDeviceCooling(batteryTemp, VehicleConstants::BAT_OPTIMAL_MIN, VehicleConstants::BAT_OPTIMAL_MAX, VehicleConstants::BAT_NORMAL_MAX, batteryAction);

        if (configuration == PowertrainConfig::AWD) {
            updateDeviceCooling(frontMotorTemp, VehicleConstants::MOTOR_OPTIMAL_MIN, VehicleConstants::MOTOR_OPTIMAL_MAX, VehicleConstants::MOTOR_NORMAL_MAX, frontMotorAction);
            updateDeviceCooling(frontInverterTemp, VehicleConstants::INV_OPTIMAL_MIN, VehicleConstants::INV_OPTIMAL_MAX, VehicleConstants::INV_NORMAL_MAX, frontInverterAction);
        } else {
            frontMotorAction = CoolingAction::TURNED_OFF;
            frontInverterAction = CoolingAction::TURNED_OFF;
        }

        // Emergency if too hot OR too cold
        bool wantsEmergency = (motorTemp > VehicleConstants::MOTOR_CRITICAL || motorTemp < VehicleConstants::MOTOR_COLD_LIMIT || // Motor too hot or too cold
                               inverterTemp > VehicleConstants::INV_CRITICAL || inverterTemp < VehicleConstants::INV_COLD_LIMIT || // Inverter too hot or too cold
                               batteryTemp > VehicleConstants::BAT_CRITICAL || batteryTemp < VehicleConstants::BAT_COLD_LIMIT);   // Battery too hot or too cold
        
        if (configuration == PowertrainConfig::AWD) {
            wantsEmergency |= (frontMotorTemp > VehicleConstants::MOTOR_CRITICAL || frontMotorTemp < VehicleConstants::MOTOR_COLD_LIMIT || 
                               frontInverterTemp > VehicleConstants::INV_CRITICAL || frontInverterTemp < VehicleConstants::INV_COLD_LIMIT);
        }

        if (wantsEmergency && !emergencyShutdown) { // Only log on transition to emergency
            if (motorTemp > VehicleConstants::MOTOR_CRITICAL) spdlog::critical("Emergency Shutdown: Motor Overheat ({} C)", motorTemp);
            if (inverterTemp > VehicleConstants::INV_CRITICAL) spdlog::critical("Emergency Shutdown: Inverter Overheat ({} C)", inverterTemp);
            if (batteryTemp > VehicleConstants::BAT_CRITICAL) spdlog::critical("Emergency Shutdown: Battery Overheat ({} C)", batteryTemp);
            if (frontMotorTemp > VehicleConstants::MOTOR_CRITICAL) spdlog::critical("Emergency Shutdown: Front Motor Overheat ({} C)", frontMotorTemp);
            if (frontInverterTemp > VehicleConstants::INV_CRITICAL) spdlog::critical("Emergency Shutdown: Front Inverter Overheat ({} C)", frontInverterTemp);
        }

        bool wantsDerate = (motorTemp > VehicleConstants::MOTOR_NORMAL_MAX || inverterTemp > VehicleConstants::INV_NORMAL_MAX || batteryTemp > VehicleConstants::BAT_NORMAL_MAX);
        if (configuration == PowertrainConfig::AWD) {
            wantsDerate |= (frontMotorTemp > VehicleConstants::MOTOR_NORMAL_MAX || frontInverterTemp > VehicleConstants::INV_NORMAL_MAX);
        }

        if (wantsDerate && !thermalDerate && !wantsEmergency) {
            QString derateReason;
            if (motorTemp > VehicleConstants::MOTOR_NORMAL_MAX) {
                derateReason += QString::asprintf("Motor (%.1f°C) ", motorTemp);
            }
            if (inverterTemp > VehicleConstants::INV_NORMAL_MAX) {
                derateReason += QString::asprintf("Inverter (%.1f°C) ", inverterTemp);
            }
            if (batteryTemp > VehicleConstants::BAT_NORMAL_MAX) {
                derateReason += QString::asprintf("Battery (%.1f°C) ", batteryTemp);
            }
            if (frontMotorTemp > VehicleConstants::MOTOR_NORMAL_MAX) derateReason += QString::asprintf("Front Motor (%.1f°C) ", frontMotorTemp);
            if (frontInverterTemp > VehicleConstants::INV_NORMAL_MAX) derateReason += QString::asprintf("Front Inverter (%.1f°C) ", frontInverterTemp);
            
            spdlog::warn("Thermal Derate Active: System exceeding normal operating thresholds. Affected units: {}", derateReason.toStdString());
        }

        // Apply 30s Hysteresis Cooldown for mode switching stability
        if (wantsEmergency != emergencyShutdown || (wantsDerate != thermalDerate && !wantsEmergency)) {
            if (modeSwitchTimer >= 30.0) {
                if (emergencyShutdown && !wantsEmergency) spdlog::info("Safety Systems Clear: Emergency Shutdown Lifted");
                if (thermalDerate && !wantsDerate) spdlog::info("Temperatures Stabilized: Thermal Derate Lifted");

                emergencyShutdown = wantsEmergency;
                if (emergencyShutdown) {
                    thermalDerate = false;
                } else {
                    thermalDerate = wantsDerate;
                }
                modeSwitchTimer = 0.0; // Lock the new state for 30 seconds
            }
        }
    }
}

void EVPowertrainSimulator::updateDeviceCooling(double temp, double optimalMin, double optimalMax, double normalMax, CoolingAction &action) {
    if (temp < optimalMin) {
        action = CoolingAction::LIQUID_WARM; // Active heating circulation
    } else if (temp <= optimalMax) {
        // De-escalate cooling toward TURNED_OFF
        if (action == CoolingAction::LIQUID_HIGH)      action = CoolingAction::LIQUID_MED;
        else if (action == CoolingAction::LIQUID_MED)  action = CoolingAction::LIQUID_LOW;
        else if (action == CoolingAction::LIQUID_WARM && temp >= optimalMin + 2.0) action = CoolingAction::TURNED_OFF; // Hysteresis
        else if (action != CoolingAction::LIQUID_WARM) action = CoolingAction::TURNED_OFF;
    } else if (temp <= normalMax) {
        // Maintain LIQUID_LOW if already cooling, otherwise allow passive soak
        if (action > CoolingAction::LIQUID_LOW) {
            action = CoolingAction::LIQUID_LOW;
        }
    } else { // temp > normalMax (Escalate)
        // Step up liquid flow rates to increase heat transfer
        if (action == CoolingAction::TURNED_OFF || action == CoolingAction::LIQUID_WARM) {
            action = CoolingAction::LIQUID_LOW;
        } else if (action == CoolingAction::LIQUID_LOW) {
            action = CoolingAction::LIQUID_MED;
        } else {
            action = CoolingAction::LIQUID_HIGH;
        }
    }
}

void EVPowertrainSimulator::updateBatteryThermalManagement() {
    // Battery Warming
    if (batteryTemp < VehicleConstants::BAT_HEATER_ON_TEMP) {
        if (!batteryHeaterOn) { spdlog::info("Battery heater ON: {:.1f}°C", batteryTemp); }
        batteryHeaterOn = true;
    } else if (batteryTemp >= VehicleConstants::BAT_OPTIMAL_MIN + 2.0) { // Hysteresis for turning off heater
        if (batteryHeaterOn) { spdlog::info("Battery heater OFF: {:.1f}°C", batteryTemp); }
        batteryHeaterOn = false;
    }

    // Battery Cooling (Chiller)
    if (batteryTemp > VehicleConstants::BAT_CHILLER_ON_TEMP) {
        if (!batteryChillerOn) { spdlog::info("Battery chiller ON: {:.1f}°C", batteryTemp); }
        batteryChillerOn = true;
    } else if (batteryTemp <= VehicleConstants::BAT_OPTIMAL_MAX - 2.0) { // Hysteresis for turning off chiller
        if (batteryChillerOn) { spdlog::info("Battery chiller OFF: {:.1f}°C", batteryTemp); }
        batteryChillerOn = false;
    }
}

void EVPowertrainSimulator::updatePTThermalManagement() {
    // PT Warming
    if (coolantPTTemp < VehicleConstants::PT_HEATER_ON_TEMP) {
        if (!ptHeaterOn) { spdlog::info("PT heater ON: {:.1f}°C", coolantPTTemp); }
        ptHeaterOn = true;
    } else if (coolantPTTemp >= VehicleConstants::MOTOR_OPTIMAL_MAX + 2.0) { // Hysteresis for turning off heater
        if (ptHeaterOn) { spdlog::info("PT heater OFF: {:.1f}°C", coolantPTTemp); }
        ptHeaterOn = false;
    }

    // PT Cooling (Chiller)
    if (coolantPTTemp > VehicleConstants::PT_CHILLER_ON_TEMP) {
        if (!ptChillerOn) { spdlog::info("PT chiller ON: {:.1f}°C", coolantPTTemp); }
        ptChillerOn = true;
    } else if (coolantPTTemp <= VehicleConstants::MOTOR_OPTIMAL_MAX - 2.0) { // Hysteresis for turning off chiller
        if (ptChillerOn) { spdlog::info("PT chiller OFF: {:.1f}°C", coolantPTTemp); }
        ptChillerOn = false;
    }
}

void EVPowertrainSimulator::updatePrecheckLogic() {
    // Once Prechecks are complete and system is Ready, we do not fall back to conditioning states while driving
    if (precheckStatus == PrecheckStatus::Ready) {
        return;
    }

    // Battery Pre-check
    if (batteryTemp < VehicleConstants::BAT_OPTIMAL_MIN) {
        precheckStatus = PrecheckStatus::HeatingBattery;
        return;
    }
    if (batteryTemp > VehicleConstants::BAT_OPTIMAL_MAX) {
        precheckStatus = PrecheckStatus::CoolingBattery;
        return;
    }

    // Powertrain Pre-check (using coolant temp as proxy for overall PT temp)
    // Note: Motor/Inverter optimal max are used here for the PT loop's "ready" state
    if (coolantPTTemp < VehicleConstants::MOTOR_COLD_LIMIT) { // Only block if below structural limits (-20C)
        precheckStatus = PrecheckStatus::HeatingPT;
        return;
    }
    if (coolantPTTemp > VehicleConstants::MOTOR_OPTIMAL_MAX + 5.0) { // Allow some buffer for PT to cool down
        precheckStatus = PrecheckStatus::CoolingPT;
        return;
    }

    // If all checks pass
    if (precheckStatus != PrecheckStatus::Ready) {
        spdlog::info("Pre-checks complete. System Ready.");
    }
    precheckStatus = PrecheckStatus::Ready;
}

void EVPowertrainSimulator::setAmbientTemp(double t) {
    ambientTemp = t;
    // Apply equilibrium only if vehicle is at rest AND has not yet started its journey
    if (powerState.speedKmh < 0.1 && powerState.distanceKm < 0.001) {
        motorTemp = t;
        inverterTemp = t;
            frontMotorTemp = t;
            frontInverterTemp = t;
        batteryTemp = t;
        coolantPTTemp = t;
        coolantBatTemp = t;
    }
}

void EVPowertrainSimulator::setSurfaceType(int typeIndex) {
    currentSurfaceType = typeIndex;
}

void EVPowertrainSimulator::setDriveMode(int index) {
    driveMode = static_cast<DriveMode>(index);
    
    std::string modeStr;
    switch(driveMode) {
        case DriveMode::ECONOMIC: modeStr = "ECONOMIC"; break;
        case DriveMode::NORMAL:   modeStr = "NORMAL"; break;
        case DriveMode::SPORT:    modeStr = "SPORT"; break;
    }
    spdlog::info("Drive Mode Switch: {}", modeStr);

    switch(driveMode) {
        case DriveMode::ECONOMIC: driveModeLimit = 0.5; break;
        case DriveMode::NORMAL:   driveModeLimit = 0.75; break;
        case DriveMode::SPORT:    driveModeLimit = 1.0; break;
    }
    // Slew rate limiter will smoothly transition to this new driveModeLimit
}

void EVPowertrainSimulator::setConfiguration(int index) {
    configuration = static_cast<PowertrainConfig>(index);
    spdlog::info("Powertrain Configuration set to: {}", (index == 0 ? "RWD" : "AWD"));
}

void EVPowertrainSimulator::setWheelConfig(int index) {
    wheelConfig = static_cast<WheelConfig>(index);
    spdlog::info("Wheel Configuration set to: {}", (index == 0 ? "Eco 18-inch" : "Staggered 21-inch"));
}