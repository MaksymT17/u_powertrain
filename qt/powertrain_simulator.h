#ifndef POWERTRAIN_SIMULATOR_H
#define POWERTRAIN_SIMULATOR_H

#include <cmath>
#include "vehicle_dto.h"
#include "power_service.h"

class EVPowertrainSimulator {
public:
    EVPowertrainSimulator();

    void update(double dt);

    // Controls
    void setThrottle(double t) { throttle = t; }
    void setBrake(double b) { brake = b; }
    void setGradient(double g) { gradient = g; }
    void setAmbientTemp(double t);
    void setWindSpeed(double w) { windSpeed = w; }
    void setDriveMode(int index);
    void setConfiguration(int index);
    void setWheelConfig(int index);
    void setIgnition(bool on);

    // State Getters
    double getSpeedKmh() const { return powerState.speedKmh; }
    double getBatteryKwh() const { return powerState.batteryKwh; }
    double getDistanceKm() const { return powerState.distanceKm; }
    double getTripTime() const { return powerState.tripTime; }
    double getPowerKw() const { return powerState.currentPowerKw; }
    double getEfficiency() const { return powerState.efficiencyKwh100; }
    double getSOC() const { return (powerState.batteryKwh / 75.0) * 100.0; }
    double getMotorTemp() const { return motorTemp; }
    double getInverterTemp() const { return inverterTemp; }
    double getFrontMotorTemp() const { return frontMotorTemp; }
    double getFrontInverterTemp() const { return frontInverterTemp; }
    CoolingAction getFrontMotorAction() const { return frontMotorAction; }
    CoolingAction getFrontInverterAction() const { return frontInverterAction; }
    double getBatteryTemp() const { return batteryTemp; }
    double getCoolantPTTemp() const { return coolantPTTemp; }
    double getCoolantBatTemp() const { return coolantBatTemp; }
    double getAmbientTemp() const { return ambientTemp; }
    double getWindSpeed() const { return windSpeed; }
    bool isEmergency() const { return emergencyShutdown; }
    bool isDerated() const { return thermalDerate; }
    DriveMode getDriveMode() const { return driveMode; }
    PowertrainConfig getConfiguration() const { return configuration; }
    bool isIgnitionOn() const { return ignitionOn; }
    double getThrottle() const { return throttle; }
    double getBrake() const { return brake; }
    double get12VPercentage() const { return (battery12VEnergy / battery12VCapacity) * 100.0; }
    PrecheckStatus getPrecheckStatus() const { return precheckStatus; }

    void setSurfaceType(int typeIndex);

    // Auxiliary Systems
    void setLowBeams(bool on) { lowBeamsOn = on; }
    void setHighBeams(bool on) { highBeamsOn = on; }
    void setInfotainmentPower(double watts) { infotainmentPowerDraw = watts; }
    void setACOn(bool on) { acOn = on; }
    void setACTargetTemp(double t) { acTargetTemp = t; }

    CoolingAction getMotorAction() const { return motorAction; }
    CoolingAction getInverterAction() const { return inverterAction; }
    CoolingAction getBatteryAction() const { return batteryAction; }

private:
    // Microservices
    PowerDeliveryService powerService;
    PowerTelemetryDTO powerState;

    double throttle = 0.0;      // 0.0 to 1.0
    double brake = 0.0;         // 0.0 to 1.0
    double gradient = 0.0;      // degrees



    int currentSurfaceType = 0;

    bool emergencyShutdown = false;
    bool thermalDerate = false;
    double smoothPowerLimit = 1.0;

    // Drive Mode & Hysteresis
    DriveMode driveMode = DriveMode::NORMAL;
    double driveModeLimit = 0.75;
    double modeSwitchTimer = 30.0; // Ready for first change

    // Powertrain Configuration
    PowertrainConfig configuration = PowertrainConfig::RWD;

    // Wheel Configuration
    WheelConfig wheelConfig = WheelConfig::STAGGERED_265_21;

    // Thermal State
    double windSpeed = 0.0;     // m/s (+ tailwind, - headwind)
    double ambientTemp = 25.0;
    double motorTemp = 25.0;
    double inverterTemp = 25.0;
    double frontMotorTemp = 25.0;
    double frontInverterTemp = 25.0;
    double batteryTemp = 25.0;
    double coolantPTTemp = 25.0;
    double coolantBatTemp = 25.0;

    CoolingAction motorAction = CoolingAction::TURNED_OFF;
    CoolingAction inverterAction = CoolingAction::TURNED_OFF;
    CoolingAction frontMotorAction = CoolingAction::TURNED_OFF;
    CoolingAction frontInverterAction = CoolingAction::TURNED_OFF;
    CoolingAction batteryAction = CoolingAction::TURNED_OFF;
    double coolingPollTimer = 0.0;
    PrecheckStatus precheckStatus = PrecheckStatus::Initializing;

    // Auxiliary Loads
    bool lowBeamsOn = false;
    bool highBeamsOn = false;
    bool ignitionOn = false;
    bool dcdcActive = false;
    bool batteryHeaterOn = false;
    double batteryHeaterPowerDraw = 3000.0; // Example: 3kW heater
    bool batteryChillerOn = false;
    double batteryChillerPowerDraw = 1000.0; // Example: 1kW chiller
    bool ptHeaterOn = false;
    double ptHeaterPowerDraw = 2000.0; // Example: 2kW heater for PT loop
    bool ptChillerOn = false;
    double ptChillerPowerDraw = 1500.0; // Example: 1.5kW chiller for PT loop
    double infotainmentPowerDraw = 150.0; // Standard compute/screen load
    bool acOn = false;
    double acTargetTemp = 22.0;

    // 12V Auxiliary System
    double battery12VEnergy = 2592000.0; // 60Ah * 12V in Joules
    const double battery12VCapacity = 2592000.0;
    const double DCDC_MAX_POWER = 1500.0; // 1.5kW DC-DC Converter
    static constexpr double SYSTEM_BASE_LOAD = 250.0; // Computers, sensors, etc.

    void updateDeviceCooling(double temp, double optimalMin, double optimalMax, double normalMax, CoolingAction &action);



    void updateBatteryThermalManagement();
    void updatePTThermalManagement();
    void updatePrecheckLogic();
};
#endif