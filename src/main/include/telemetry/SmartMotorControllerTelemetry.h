#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
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

    void Publish(std::shared_ptr<nt::NetworkTable> publishTable, TelemetryVerbosity verbosity);

    // Public telemetry data
    bool mechanismLowerLimit = false;
    bool mechanismUpperLimit = false;
    bool temperatureLimit = false;
    bool velocityControl = false;
    bool elevatorFeedforward = false;
    bool armFeedforward = false;
    bool simpleFeedforward = false;
    bool motionProfile = false;
    double setpointPosition = 0;
    double setpointVelocity = 0;
    double feedforwardVoltage = 0;
    double pidOutputVoltage = 0;
    double outputVoltage = 0;
    double statorCurrent = 0;
    units::temperature::fahrenheit_t temperature = 72_degF;
    units::length::meter_t distance = 0_m;
    units::velocity::meters_per_second_t linearVelocity = 0_mps;
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