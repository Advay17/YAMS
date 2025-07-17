#ifndef SMART_MOTOR_CONTROLLER_CONFIG_H
#define SMART_MOTOR_CONTROLLER_CONFIG_H

#include <optional>
#include <string>
#include <vector>
#include <utility> // For std::pair
#include <frc2/command/Subsystem.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include "gearing/MechanismGearing.h"

template <class Distance>
  requires units::length_unit<Distance> || units::angle_unit<Distance>
class SmartMotorControllerConfig {
public:
    enum class TelemetryVerbosity {
        LOW, MID, HIGH
    };

    enum class MotorMode {
        BRAKE, COAST
    };

    enum class ControlMode {
        OPEN_LOOP, CLOSED_LOOP
    };

    explicit SmartMotorControllerConfig(frc2::Subsystem* subsystem);

    SmartMotorControllerConfig& withExternalEncoderInverted(bool externalEncoderInverted);
    SmartMotorControllerConfig& withControlMode(ControlMode controlMode);
    SmartMotorControllerConfig& withClosedLoopControllerMaximumVoltage(double volts);
    SmartMotorControllerConfig& withStartingPosition(double startingAngle);
    SmartMotorControllerConfig& withUseExternalFeedbackEncoder(bool useExternalEncoder);
    SmartMotorControllerConfig& withEncoderInverted(bool inverted);
    SmartMotorControllerConfig& withMotorInverted(bool motorInverted);
    SmartMotorControllerConfig& withTemperatureCutoff(double cutoff);
    SmartMotorControllerConfig& withZeroOffset(double distance);
    SmartMotorControllerConfig& withContinuousWrapping(double bottom, double top);
    SmartMotorControllerConfig& withFeedforward(frc::ArmFeedforward* armFeedforward);
    SmartMotorControllerConfig& withFeedforward(frc::ElevatorFeedforward* elevatorFeedforward);
    SmartMotorControllerConfig& withFeedforward(frc::SimpleMotorFeedforward<Distance>* simpleFeedforward);
    SmartMotorControllerConfig& withClosedLoopController(frc::ProfiledPIDController<Distance>* controller);
    SmartMotorControllerConfig& withClosedLoopController(double kP, double kI, double kD);

    // Getters
    std::optional<int> getStatorStallCurrentLimit() const;
    std::optional<int> getSupplyStallCurrentLimit() const;
    std::optional<double> getVoltageCompensation() const;
    std::optional<MotorMode> getIdleMode() const;
    std::optional<double> getMechanismLowerLimit() const;
    std::optional<double> getMechanismUpperLimit() const;
    std::optional<std::string> getTelemetryName() const;
    std::optional<TelemetryVerbosity> getVerbosity() const;
    frc2::Subsystem* getSubsystem() const;

private:
    frc2::Subsystem* subsystem;
    std::optional<void*> externalEncoder = {};
    bool externalEncoderInverted = false;
    std::vector<std::pair<void*, bool>> followers;
    std::optional<frc::SimpleMotorFeedforward<Distance>*> simpleFeedforward = {};
    std::optional<frc::ArmFeedforward*> armFeedforward = {};
    std::optional<frc::ElevatorFeedforward*> elevatorFeedforward = {};
    std::optional<frc::ProfiledPIDController<Distance>*> controller = {};
    std::optional<frc::PIDController*> simpleController = {};
    double gearing = 1.0;
    MechanismGearing externalEncoderGearing;
    std::optional<double> mechanismCircumference;
    double controlPeriod = 20.0;
    double openLoopRampRate = 0.0;
    double closeLoopRampRate = 0.0;
    std::optional<int> statorStallCurrentLimit = {};
    std::optional<int> supplyStallCurrentLimit = {};
    std::optional<double> voltageCompensation = {};
    std::optional<MotorMode> idleMode = {};
    std::optional<units::radian_t> mechanismLowerLimit = {};
    std::optional<units::radian_t> mechanismUpperLimit = {};
    std::optional<std::string> telemetryName = {};
    std::optional<TelemetryVerbosity> verbosity = {};
    std::optional<double> zeroOffset = {};
    std::optional<units::celsius_t> temperatureCutoff = {};
    bool encoderInverted = false;
    bool motorInverted = false;
    bool useExternalEncoder = true;
    std::optional<units::radian_t> startingPosition = {};
    std::optional<double> closedLoopControllerMaximumVoltage = {};
    std::optional<units::radian_t> feedbackSynchronizationThreshold = {};
    ControlMode motorControllerMode = ControlMode::CLOSED_LOOP;
    std::optional<units::radian_t> maxDiscontinuityPoint = {};
    std::optional<units::radian_t> minDiscontinuityPoint = {};
};

#endif // SMART_MOTOR_CONTROLLER_CONFIG_H