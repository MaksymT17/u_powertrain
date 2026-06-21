#include "power_service.h"
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>

void PowerDeliveryService::resetState() {
    speed = 0.0;
    batteryEnergy = VehicleConstants::BATTERY_CAPACITY_J;
    distance = 0.0;
    tripTime = 0.0;
    effPowerSum = 0.0;
    effSpeedSum = 0.0;
    effTicks = 0;
}

PowerTelemetryDTO PowerDeliveryService::calculatePhysics(const VehicleCommandDTO& inputs, const ThermalTelemetryDTO& thermals, double dt) {
    tripTime += dt;

    // 1. Environment & Friction Profile
    double airDensity = AIR_PRESSURE_SEA_LEVEL / (SPECIFIC_GAS_CONST_AIR * (inputs.ambientTemp + 273.15));
    
    double dynamicWheelRadius = VehicleConstants::WHEEL_RADIUS_18;
    double front_crr = VehicleConstants::CRR_18_FRONT;
    double rear_crr = VehicleConstants::CRR_18_REAR;
    double currentDragCoeffMultiplier = 1.0;
    double mu_friction = VehicleConstants::MU_ASPHALT_18;

    if (inputs.wheelConfig == WheelConfig::STAGGERED_265_21) {
        dynamicWheelRadius = VehicleConstants::WHEEL_RADIUS_21;
        front_crr = VehicleConstants::CRR_21_FRONT;
        rear_crr = VehicleConstants::CRR_21_REAR;
        mu_friction = VehicleConstants::MU_ASPHALT_21;
        currentDragCoeffMultiplier = VehicleConstants::DRAG_COEFF_21 / VehicleConstants::DRAG_COEFF_18;
    }

    if (inputs.surfaceType == 1) { // Gravel
        front_crr = rear_crr = VehicleConstants::CRR_GRAVEL;
        currentDragCoeffMultiplier = VehicleConstants::DRAG_MULT_GRAVEL;
        mu_friction = VehicleConstants::MU_GRAVEL;
    } else if (inputs.surfaceType == 2) { // Ice
        front_crr = rear_crr = VehicleConstants::CRR_ICE;
        mu_friction = VehicleConstants::MU_ICE;
    }

    // 2. Drive Mode & Derating Logic
    double modeMultiplier = thermals.emergencyShutdown ? 0.20 : (thermals.thermalDerate ? 0.70 : 1.0);
    if (inputs.driveMode == DriveMode::ECONOMIC) modeMultiplier *= 0.5;
    else if (inputs.driveMode == DriveMode::NORMAL) modeMultiplier *= 0.75;

    if (!inputs.ignitionOn || !inputs.isPrecheckReady) modeMultiplier = 0.0;

    // Slew Rate Limiter (Smooth power transition)
    double maxStep = 0.1 * dt;
    if (smoothLimit < modeMultiplier) smoothLimit = std::min(modeMultiplier, smoothLimit + maxStep);
    else if (smoothLimit > modeMultiplier) smoothLimit = std::max(modeMultiplier, smoothLimit - maxStep);

    double available_battery_power = VehicleConstants::MAX_POWER_W - inputs.hvAuxLoadW;
    double propulsion_limit = std::max(0.0, available_battery_power * 0.94);

    // 3. Virtual Slip Control (TCS) & Torque Vectoring
    double angular_vel = speed / dynamicWheelRadius;
    double motor_rpm = (angular_vel * 60.0 / (2.0 * M_PI)) * 9.04;
    
    double active_max_torque = VehicleConstants::REAR_MOTOR_MAX_TORQUE_NM + (inputs.configuration == PowertrainConfig::AWD ? VehicleConstants::FRONT_MOTOR_MAX_TORQUE_NM : 0.0);
    double total_avail_torque = std::min(active_max_torque, propulsion_limit / std::max(0.1, speed / dynamicWheelRadius));

    if (motor_rpm > 17800.0) total_avail_torque *= std::max(0.0, (18000.0 - motor_rpm) / 200.0);

    double target_engine_torque = inputs.throttle * smoothLimit * total_avail_torque;
    
    // --- VIRTUAL SLIP CONTROL MATH ---
    // Calculate normal force and maximum transferable torque before tires spin
    double mass = (inputs.configuration == PowertrainConfig::AWD) ? VehicleConstants::AWD_MASS_KG : VehicleConstants::RWD_MASS_KG;
    double normal_force = mass * 9.81 * std::cos(inputs.gradient * M_PI / 180.0);
    double rear_weight_bias = (inputs.configuration == PowertrainConfig::AWD) ? VehicleConstants::AWD_WEIGHT_BIAS_REAR : VehicleConstants::RWD_WEIGHT_BIAS_REAR;
    
    double max_grip_torque_rear = (mu_friction * normal_force * rear_weight_bias) * dynamicWheelRadius;
    double max_grip_torque_front = (mu_friction * normal_force * (1.0 - rear_weight_bias)) * dynamicWheelRadius;

    double rear_engine_torque = std::min({target_engine_torque, VehicleConstants::REAR_MOTOR_MAX_TORQUE_NM, max_grip_torque_rear});
    double front_engine_torque = 0.0;

    if (inputs.configuration == PowertrainConfig::AWD) {
        // Dynamic Vectoring: If rear is saturated by slip limit, transfer excess demand to the front motor
        double unmet_torque = target_engine_torque - rear_engine_torque;
        front_engine_torque = std::min({unmet_torque, VehicleConstants::FRONT_MOTOR_MAX_TORQUE_NM, max_grip_torque_front});
    }
    double engine_force = (rear_engine_torque + front_engine_torque) / dynamicWheelRadius;

    // 4. Resistance & Braking (Regen Vectoring)
    double v_rel = speed - inputs.windSpeed;
    double drag_force = 0.5 * (BASE_DRAG_COEFF * currentDragCoeffMultiplier) * FRONTAL_AREA * airDensity * v_rel * std::abs(v_rel);
    
    // Split rolling resistance mathematically by weight distribution on the tires
    double rolling_force = (front_crr * normal_force * (1.0 - rear_weight_bias)) + (rear_crr * normal_force * rear_weight_bias);
    double gradient_force = mass * 9.81 * std::sin(inputs.gradient * M_PI / 180.0);

    double active_max_regen = VehicleConstants::REAR_MAX_REGEN_TORQUE_NM + (inputs.configuration == PowertrainConfig::AWD ? VehicleConstants::FRONT_MAX_REGEN_TORQUE_NM : 0.0);
    double brake_input = (!inputs.ignitionOn || !inputs.isPrecheckReady) ? 1.0 : inputs.brake;
    
    double target_regen_torque = std::min(brake_input, 0.5) / 0.5 * active_max_regen;
    double front_regen_torque = 0.0;
    double rear_regen_torque = target_regen_torque;

    if (inputs.configuration == PowertrainConfig::AWD) {
        front_regen_torque = std::min(target_regen_torque, VehicleConstants::FRONT_MAX_REGEN_TORQUE_NM);
        rear_regen_torque = std::max(0.0, target_regen_torque - front_regen_torque);
    }

    // Slip Control for Regen (Prevent locking the wheels on ice)
    front_regen_torque = std::min(front_regen_torque, max_grip_torque_front);
    rear_regen_torque = std::min(rear_regen_torque, max_grip_torque_rear);

    double regen_force = (rear_regen_torque + front_regen_torque) / dynamicWheelRadius;
    double mech_brake_force = std::max(0.0, brake_input - 0.5) / 0.5 * 8.0 * mass;

    // 5. Physics Integration
    double net_force = engine_force - drag_force - rolling_force - regen_force - mech_brake_force - gradient_force;
    speed = std::max(0.0, speed + (net_force / mass) * dt);
    distance += speed * dt;

    // 6. Efficiency & Heat Generation
    double rear_mech_power = rear_engine_torque * (speed / dynamicWheelRadius);
    double front_mech_power = front_engine_torque * (speed / dynamicWheelRadius);
    double motor_rpm_norm = std::min(motor_rpm / 18000.0, 1.0);
    
    double motor_eff = std::max(0.70, MOTOR_ETA_BASE - (MOTOR_K_COPPER * std::pow(inputs.throttle, 2)) - (MOTOR_K_IRON * motor_rpm_norm));
    double inv_eff = std::max(0.85, INV_ETA_BASE - (INV_K_SWITCH * inputs.throttle));

    double electrical_draw = (rear_mech_power / (motor_eff * inv_eff)) + inputs.hvAuxLoadW;
    if (inputs.configuration == PowertrainConfig::AWD && front_mech_power > 0) {
        electrical_draw += (front_mech_power / (motor_eff * inv_eff)); // Assuming identical efficiency curves for prototype
    }

    double battery_current = electrical_draw / (320.0 + (100.0 * (batteryEnergy / VehicleConstants::BATTERY_CAPACITY_J)));
    state.batteryHeatW = std::pow(battery_current, 2) * 0.042;

    double rear_regen_power = (rear_regen_torque / dynamicWheelRadius) * speed;
    double front_regen_power = (front_regen_torque / dynamicWheelRadius) * speed;
    double regen_returned = (rear_regen_power + front_regen_power) * 0.85;

    batteryEnergy = std::clamp(batteryEnergy - (electrical_draw * dt) + (regen_returned * dt), 0.0, VehicleConstants::BATTERY_CAPACITY_J);
    
    state.rearMotorHeatW = (std::abs(rear_mech_power) + std::abs(rear_regen_power)) * (1.0 - motor_eff);
    state.rearInverterHeatW = (std::abs(rear_mech_power) + std::abs(rear_regen_power)) * (1.0 - inv_eff);
    
    state.frontMotorHeatW = (inputs.configuration == PowertrainConfig::AWD) ? (std::abs(front_mech_power) + std::abs(front_regen_power)) * (1.0 - motor_eff) : 0.0;
    state.frontInverterHeatW = (inputs.configuration == PowertrainConfig::AWD) ? (std::abs(front_mech_power) + std::abs(front_regen_power)) * (1.0 - inv_eff) : 0.0;

    // 7. Telemetry Averages
    state.currentPowerKw = (electrical_draw - regen_returned) / 1000.0;
    effPowerSum += std::max(0.0, state.currentPowerKw);
    effSpeedSum += speed * 3.6;
    effTicks++;
    if (effTicks > 600) {
        effPowerSum *= (600.0 / 601.0);
        effSpeedSum *= (600.0 / 601.0);
        effTicks = 600;
    }
    state.efficiencyKwh100 = (effSpeedSum / std::max(1, effTicks) > 2.0) ? (effPowerSum / effSpeedSum * 100.0) : 0.0;
    
    state.speedKmh = speed * 3.6;
    state.distanceKm = distance / 1000.0;
    state.tripTime = tripTime;
    state.batteryKwh = batteryEnergy / 3600000.0;

    return state;
}