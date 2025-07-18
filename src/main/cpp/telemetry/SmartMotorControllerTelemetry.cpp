#include "telemetry/SmartMotorControllerTelemetry.h"

void SmartMotorControllerTelemetry::Publish(std::shared_ptr<nt::NetworkTable> publishTable, TelemetryVerbosity verbosity)
{
    if (publishTable != table)
    {
        table = publishTable;
        mechanismLowerLimitPublisher = table->GetBooleanTopic("Mechanism Lower Limit").Publish();
        mechanismUpperLimitPublisher = table->GetBooleanTopic("Mechanism Upper Limit").Publish();
        temperatureLimitPublisher = table->GetBooleanTopic("Temperature Limit").Publish();
        velocityControlPublisher = table->GetBooleanTopic("Velocity Control").Publish();
        elevatorFeedforwardPublisher = table->GetBooleanTopic("Elevator Feedforward").Publish();
        armFeedforwardPublisher = table->GetBooleanTopic("Arm Feedforward").Publish();
        simpleFeedforwardPublisher = table->GetBooleanTopic("Simple Feedforward").Publish();
        motionProfilePublisher = table->GetBooleanTopic("Motion Profile").Publish();
        setpointPositionPublisher = table->GetDoubleTopic("Setpoint Position (Rotations)").Publish();
        setpointVelocityPublisher = table->GetDoubleTopic("Setpoint Velocity (Rotations per Second)").Publish();
        feedforwardVoltagePublisher = table->GetDoubleTopic("Feedforward Voltage").Publish();
        pidOutputVoltagePublisher = table->GetDoubleTopic("PID Output (Voltage)").Publish();
        outputVoltagePublisher = table->GetDoubleTopic("Motor Output Voltage").Publish();
        statorCurrentPublisher = table->GetDoubleTopic("Stator Current (Amps)").Publish();
        temperaturePublisher = table->GetDoubleTopic("Temperature (Celsius)").Publish();
        measurementPositionPublisher = table->GetDoubleTopic("Measurement Position (Meters)").Publish();
        measurementVelocityPublisher = table->GetDoubleTopic("Measurement Velocity (Meters per Second)").Publish();
        mechanismPositionPublisher = table->GetDoubleTopic("Mechanism Position (Rotations)").Publish();
        mechanismVelocityPublisher = table->GetDoubleTopic("Mechanism Velocity (Rotations per Second)").Publish();
        rotorPositionPublisher = table->GetDoubleTopic("Rotor Position (Rotations)").Publish();
        rotorVelocityPublisher = table->GetDoubleTopic("Rotor Velocity (Rotations per Second)").Publish();
    }

    if (table)
    {
        mechanismLowerLimitPublisher.Set(mechanismLowerLimit);
        mechanismUpperLimitPublisher.Set(mechanismUpperLimit);
        temperatureLimitPublisher.Set(temperatureLimit);
        velocityControlPublisher.Set(velocityControl);
        elevatorFeedforwardPublisher.Set(elevatorFeedforward);
        armFeedforwardPublisher.Set(armFeedforward);
        simpleFeedforwardPublisher.Set(simpleFeedforward);
        motionProfilePublisher.Set(motionProfile);
        setpointPositionPublisher.Set(setpointPosition);
        setpointVelocityPublisher.Set(setpointVelocity);
        feedforwardVoltagePublisher.Set(feedforwardVoltage);
        pidOutputVoltagePublisher.Set(pidOutputVoltage);
        outputVoltagePublisher.Set(outputVoltage);
        statorCurrentPublisher.Set(statorCurrent);
        temperaturePublisher.Set(((units::temperature::celsius_t) temperature).value());
        measurementPositionPublisher.Set(((units::length::meter_t) distance).value());
        measurementVelocityPublisher.Set(((units::velocity::meters_per_second_t) linearVelocity).value());
        mechanismPositionPublisher.Set(((units::angle::turn_t) mechanismPosition).value());
        mechanismVelocityPublisher.Set(((units::angular_velocity::turns_per_second_t) mechanismVelocity).value());
        rotorPositionPublisher.Set(((units::angle::turn_t) rotorPosition).value());
        rotorVelocityPublisher.Set(((units::angular_velocity::turns_per_second_t) rotorVelocity).value());
    }
}