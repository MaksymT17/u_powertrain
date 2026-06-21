#ifndef VEHICLE_DTO_H
#define VEHICLE_DTO_H

// --------------------------------------------------------------------------
// SYSTEM CONSTANTS
// --------------------------------------------------------------------------
namespace VehicleConstants {
    // Battery Capacity & Power Limits
    constexpr double BATTERY_CAPACITY_KWH = 75.0;
    constexpr double BATTERY_CAPACITY_J = BATTERY_CAPACITY_KWH * 3600000.0;
    constexpr double MAX_POWER_W = 210000.0;
    constexpr double MAX_AWD_DISPLAY_POWER_W = 270000.0;

    // Weight and Dimensions
    constexpr double RWD_MASS_KG = 1850.0;
    constexpr double AWD_MASS_KG = 1930.0;
    constexpr double RWD_WEIGHT_BIAS_REAR = 0.52;
    constexpr double AWD_WEIGHT_BIAS_REAR = 0.50;

    // Torque Specifications
    constexpr double REAR_MOTOR_MAX_TORQUE_NM = 3600.0;
    constexpr double FRONT_MOTOR_MAX_TORQUE_NM = 1200.0;
    constexpr double REAR_MAX_REGEN_TORQUE_NM = 1800.0;
    constexpr double FRONT_MAX_REGEN_TORQUE_NM = 1200.0;

    // Wheel Configurations
    constexpr double WHEEL_RADIUS_18 = 0.33;
    constexpr double WHEEL_RADIUS_21 = 0.36;
    constexpr double DRAG_COEFF_18 = 0.25;
    constexpr double DRAG_COEFF_21 = 0.255;
    constexpr double CRR_18_FRONT = 0.012;
    constexpr double CRR_18_REAR = 0.012;
    constexpr double CRR_21_FRONT = 0.008;
    constexpr double CRR_21_REAR = 0.010;
    constexpr double MU_ASPHALT_18 = 0.85;
    constexpr double MU_ASPHALT_21 = 0.98;

    // Surface Parameters
    constexpr double CRR_GRAVEL = 0.025;
    constexpr double DRAG_MULT_GRAVEL = 1.05;
    constexpr double MU_GRAVEL = 0.45;
    constexpr double CRR_ICE = 0.005;
    constexpr double MU_ICE = 0.15;

    // Thermal Mass Definitions (J/C)
    constexpr double MOTOR_THERMAL_MASS = 22000.0;
    constexpr double INVERTER_THERMAL_MASS = 3400.0;
    constexpr double FRONT_MOTOR_THERMAL_MASS = 15000.0;
    constexpr double FRONT_INVERTER_THERMAL_MASS = 2500.0;
    constexpr double BATTERY_THERMAL_MASS = 309000.0;
    constexpr double COOLANT_PT_MASS = 33700.0;
    constexpr double COOLANT_BAT_MASS = 22400.0;

    // Heat Coefficients (W/C)
    constexpr double COOLANT_PT_RAD_H = 80.0;
    constexpr double COOLANT_BAT_RAD_H = 120.0;
    constexpr double NATURAL_CONVECTION = 5.0;
    constexpr double LIQUID_COOLING_WARM = 60.0;
    constexpr double LIQUID_COOLING_LOW = 120.0;
    constexpr double LIQUID_COOLING_MED = 250.0;
    constexpr double LIQUID_COOLING_HIGH = 450.0;
    constexpr double RAM_AIR_K = 0.06;

    // Thermal Limits (C)
    constexpr double MOTOR_OPTIMAL_MIN = 40.0;
    constexpr double MOTOR_OPTIMAL_MAX = 80.0;
    constexpr double MOTOR_NORMAL_MAX = 90.0;
    constexpr double MOTOR_CRITICAL = 140.0;
    constexpr double MOTOR_COLD_LIMIT = -20.0;

    constexpr double INV_OPTIMAL_MIN = 35.0;
    constexpr double INV_OPTIMAL_MAX = 65.0;
    constexpr double INV_NORMAL_MAX = 75.0;
    constexpr double INV_CRITICAL = 120.0;
    constexpr double INV_COLD_LIMIT = -20.0;

    constexpr double BAT_OPTIMAL_MIN = 20.0;
    constexpr double BAT_OPTIMAL_MAX = 35.0;
    constexpr double BAT_NORMAL_MAX = 45.0;
    constexpr double BAT_CRITICAL = 50.0;
    constexpr double BAT_COLD_LIMIT = -10.0;

    // Heating & Cooling Target Thresholds (C)
    constexpr double BAT_HEATER_ON_TEMP = 18.0;
    constexpr double BAT_CHILLER_ON_TEMP = 37.0;
    constexpr double PT_HEATER_ON_TEMP = 38.0;
    constexpr double PT_CHILLER_ON_TEMP = 82.0;
}

// --------------------------------------------------------------------------
// SYSTEM ENUMERATIONS
// --------------------------------------------------------------------------
enum class CoolingAction {
    LIQUID_WARM = -1,
    TURNED_OFF = 0,
    LIQUID_LOW = 1,
    LIQUID_MED = 2,
    LIQUID_HIGH = 3
};

enum class PrecheckStatus {
    Initializing = 0,
    HeatingBattery = 1,
    CoolingBattery = 2,
    HeatingPT = 3,
    CoolingPT = 4,
    Ready = 5
};

enum class PowertrainConfig {
    RWD = 0,
    AWD = 1
};

enum class DriveMode {
    ECONOMIC = 0,
    NORMAL = 1,
    SPORT = 2
};

enum class WheelConfig {
    ECO_235_18 = 0,
    STAGGERED_265_21 = 1
};

// --------------------------------------------------------------------------
// DATA TRANSFER OBJECTS (DTOs)
// In a microservice architecture, these represent the exact JSON/Protobuf 
// messages passed over the network between isolated services.
// --------------------------------------------------------------------------

// 1. Command Message (From UI/Driver to Services)
struct VehicleCommandDTO {
    double throttle = 0.0;
    double brake = 0.0;
    double gradient = 0.0;
    bool ignitionOn = false;
    PowertrainConfig configuration = PowertrainConfig::AWD;
    DriveMode driveMode = DriveMode::SPORT;
    double ambientTemp = 25.0;
    double windSpeed = 0.0;
    int surfaceType = 0; // 0=Asphalt, 1=Gravel, 2=Ice
    double hvAuxLoadW = 0.0;
    bool isPrecheckReady = false;
    WheelConfig wheelConfig = WheelConfig::STAGGERED_265_21;
};

// 2. Power & Dynamics Telemetry (From Math Service to Orchestrator/UI)
struct PowerTelemetryDTO {
    double speedKmh = 0.0;
    double distanceKm = 0.0;
    double tripTime = 0.0;
    double batteryKwh = 75.0;
    double currentPowerKw = 0.0;
    double efficiencyKwh100 = 0.0;
    
    // Heat generation exported exclusively for the Thermal Service to consume
    double rearMotorHeatW = 0.0;
    double rearInverterHeatW = 0.0;
    double frontMotorHeatW = 0.0;
    double frontInverterHeatW = 0.0;
    double batteryHeatW = 0.0;
};

// 3. Thermal Telemetry (From Thermal Service to Orchestrator/UI)
struct ThermalTelemetryDTO {
    double motorTemp = 25.0, inverterTemp = 25.0;
    double frontMotorTemp = 25.0, frontInverterTemp = 25.0;
    double batteryTemp = 25.0, coolantPTTemp = 25.0, coolantBatTemp = 25.0;
    
    CoolingAction motorAction = CoolingAction::TURNED_OFF;
    CoolingAction inverterAction = CoolingAction::TURNED_OFF;
    CoolingAction frontMotorAction = CoolingAction::TURNED_OFF;
    CoolingAction frontInverterAction = CoolingAction::TURNED_OFF;
    CoolingAction batteryAction = CoolingAction::TURNED_OFF;

    bool emergencyShutdown = false;
    bool thermalDerate = false;
    PrecheckStatus precheckStatus = PrecheckStatus::Initializing;
};

#endif