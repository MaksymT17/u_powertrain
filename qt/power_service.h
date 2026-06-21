#ifndef POWER_SERVICE_H
#define POWER_SERVICE_H

#include "vehicle_dto.h"

// --------------------------------------------------------------------------
// POWER & DYNAMICS MICROSERVICE (Isolated Math Engine)
// --------------------------------------------------------------------------
class PowerDeliveryService {
public:
    PowerDeliveryService() = default;

    // Resets internal accumulators (used on application start/reset)
    void resetState();
    PowerTelemetryDTO getState() const { return state; }

    // Takes inputs and thermal limits, computes new physical state
    PowerTelemetryDTO calculatePhysics(
        const VehicleCommandDTO& inputs, 
        const ThermalTelemetryDTO& thermals, 
        double dt);
        
private:
    // Internal state isolated from the rest of the application
    PowerTelemetryDTO state;

    double speed = 0.0;         // m/s
    double batteryEnergy = 75.0 * 3600000.0; // Joules (75 kWh)
    double distance = 0.0;      // meters
    double tripTime = 0.0;      // seconds
    double smoothLimit = 1.0;

    double effPowerSum = 0.0;
    double effSpeedSum = 0.0;
    int effTicks = 0;

    // Vehicle Dynamic Constants
    static constexpr double FRONTAL_AREA = 2.2;
    static constexpr double BASE_DRAG_COEFF = 0.25;

    // Efficiency Coefficients
    static constexpr double MOTOR_ETA_BASE = 0.95;
    static constexpr double MOTOR_K_COPPER = 0.08;
    static constexpr double MOTOR_K_IRON   = 0.04;
    static constexpr double INV_ETA_BASE   = 0.98;
    static constexpr double INV_K_SWITCH   = 0.03;

    static constexpr double AIR_PRESSURE_SEA_LEVEL = 101325.0;
    static constexpr double SPECIFIC_GAS_CONST_AIR = 287.05;
};

#endif