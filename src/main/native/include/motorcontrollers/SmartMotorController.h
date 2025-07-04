#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/BooleanPublisher.h>
#include <networktables/DoublePublisher.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/temperature.h>
#include <units/length.h>
#include <units/velocity.h>
#include <memory>

class SmartMotorControllerTelemetry {
public:
    SmartMotorControllerTelemetry();

    enum class TelemetryVerbosity {
        LOW,
        MEDIUM,
        HIGH
    };

    void publish(std::shared_ptr<nt::NetworkTable> publishTable, TelemetryVerbosity verbosity);

    // Public telemetry data
    bool mechanismLowerLimit;
    bool mechanismUpperLimit;
    bool temperatureLimit;
    bool velocityControl;
    bool elevatorFeedforward;
    bool armFeedforward;
    bool simpleFeedforward;
    bool motionProfile;
    double setpointPosition;
    double setpointVelocity;
    double feedforwardVoltage;
    double pidOutputVoltage;
    double outputVoltage;
    double statorCurrent;
    units::temperature::fahrenheit_t temperature;
    units::length::meter_t distance;
    units::velocity::meters_per_second_t linearVelocity;
    units::angle::radian_t mechanismPosition;
    units::angular_velocity::radians_per_second_t mechanismVelocity;
    units::angle::radian_t rotorPosition;
    units::angular_velocity::radians_per_second_t rotorVelocity;

private:
    std::shared_ptr<nt::NetworkTable> table;

    nt::BooleanPublisher mechanismLowerLimitPublisher;
    nt::BooleanPublisher mechanismUpperLimitPublisher;
    nt::BooleanPublisher temperatureLimitPublisher;
    nt::BooleanPublisher velocityControlPublisher;
    nt::BooleanPublisher elevatorFeedforwardPublisher;
    nt::BooleanPublisher armFeedforwardPublisher;
    nt::BooleanPublisher simpleFeedforwardPublisher;
    nt::BooleanPublisher motionProfilePublisher;
    nt::DoublePublisher setpointPositionPublisher;
    nt::DoublePublisher setpointVelocityPublisher;
    nt::DoublePublisher feedforwardVoltagePublisher;
    nt::DoublePublisher pidOutputVoltagePublisher;
    nt::DoublePublisher outputVoltagePublisher;
    nt::DoublePublisher statorCurrentPublisher;
    nt::DoublePublisher temperaturePublisher;
    nt::DoublePublisher measurementPositionPublisher;
    nt::DoublePublisher measurementVelocityPublisher;
    nt::DoublePublisher mechanismPositionPublisher;
    nt::DoublePublisher mechanismVelocityPublisher;
    nt::DoublePublisher rotorPositionPublisher;
    nt::DoublePublisher rotorVelocityPublisher;
};