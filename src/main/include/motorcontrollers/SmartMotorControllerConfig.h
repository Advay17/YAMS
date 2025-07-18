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

class SmartMotorControllerConfig
{
public:
    using Velocity =
        units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<Velocity>;
    using Acceleration =
        units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using Acceleration_t = units::unit_t<Acceleration>;
    enum class TelemetryVerbosity
    {
        LOW,
        MID,
        HIGH
    };

    enum class MotorMode
    {
        BRAKE,
        COAST
    };

    enum class ControlMode
    {
        OPEN_LOOP,
        CLOSED_LOOP
    };

    explicit SmartMotorControllerConfig(frc2::Subsystem *subsystem)
        : subsystem(subsystem) {}

    SmartMotorControllerConfig &WithExternalEncoderInverted(bool externalEncoderInverted)
    {
        this->externalEncoderInverted = externalEncoderInverted;
        return *this;
    }
    SmartMotorControllerConfig &WithControlMode(ControlMode controlMode)
    {
        this->motorControllerMode = controlMode;
        return *this;
    }
    SmartMotorControllerConfig &WithFeedbackSynchronizationThreshold(units::radian_t angle)
    {
        if (mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        this->feedbackSynchronizationThreshold = angle;
        return *this;
    }
    SmartMotorControllerConfig &WithClosedLoopControllerMaximumVoltage(units::volt_t volts)
    {
        this->closedLoopControllerMaximumVoltage = volts;
        return *this;
    }
    SmartMotorControllerConfig &WithStartingPosition(units::radian_t angle)
    {
        this->startingPosition = angle;
        return *this;
    }

    SmartMotorControllerConfig &WithStartingPosition(units::meter_t startingAngle)
    {
        return withStartingPosition(ConvertToMechanism(startingAngle));
    }

    SmartMotorControllerConfig &WithUseExternalFeedbackEncoder(bool useExternalEncoder)
    {
        this->useExternalEncoder = useExternalEncoder;
        return *this;
    }
    SmartMotorControllerConfig &WithEncoderInverted(bool inverted)
    {
        this->encoderInverted = inverted;
        return *this;
    }
    SmartMotorControllerConfig &WithMotorInverted(bool motorInverted)
    {
        this->motorInverted = motorInverted;
        return *this;
    }
    SmartMotorControllerConfig &WithTemperatureCutoff(units::celsius_t cutoff)
    {
        this->temperatureCutoff = cutoff;
        return *this;
    }
    SmartMotorControllerConfig &WithZeroOffset(units::meter_t distance)
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        zeroOffset = ConvertToMechanism(distance);
        return *this;
    }
    SmartMotorControllerConfig &WithZeroOffset(units::radian_t angle)
    {
        zeroOffset = angle;
        return *this;
    }
    SmartMotorControllerConfig &WithContinuousWrapping(units::turn_t bottom, units::turn_t top)
    {
        if (mechanismUpperLimit.has_value() || mechanismLowerLimit.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        if (mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        if (simpleController.has_value())
            simpleController.value()->EnableContinuousInput(bottom.value(), top.value());
        if (controller.has_value())
            controller.value()->EnableContinuousInput(bottom.value(), top.value());
        if (!simpleController.has_value() && !controller.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        maxDiscontinuityPoint = top;
        minDiscontinuityPoint = bottom;
        return *this;
    }
    SmartMotorControllerConfig &WithClosedLoopTolerance(units::turn_t tolerance)
    {
        if (controller)
            controller.value()->SetTolerance(tolerance.value());
        if (simpleController)
            simpleController.value()->SetTolerance(tolerance.value());
        if (!controller.has_value() && simpleController.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return this;
    }
    SmartMotorControllerConfig &WithClosedLoopTolerance(units::meter_t tolerance)
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        if (controller)
            controller.value()->SetTolerance(tolerance.value());
        if (simpleController)
            simpleController.value()->SetTolerance(tolerance.value());
        if (!controller.has_value() && simpleController.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return this;
    }
    SmartMotorControllerConfig &WithTelemetry(std::string telemetryName, TelemetryVerbosity verbosity)
    {
        this->telemetryName = telemetryName;
        this->verbosity = verbosity;
        return *this;
    }
    SmartMotorControllerConfig &WithTelemetry(TelemetryVerbosity verbosity)
    {
        this->telemetryName = "motor";
        this->verbosity = verbosity;
        return *this;
    }
    SmartMotorControllerConfig &WithSoftLimit(units::meter_t low, units::meter_t high)
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        mechanismLowerLimit = ConvertToMechanism(low);
        mechanismUpperLimit = ConvertToMechanism(high);
        return *this;
    }
    SmartMotorControllerConfig &WithSoftLimit(units::radian_t low, units::radian_t high)
    {
        mechanismLowerLimit = low;
        mechanismUpperLimit = high;
        return *this;
    }
    SmartMotorControllerConfig &WithIdleMode(MotorMode *idleMode)
    {
        this->idleMode = idleMode;
        return *this;
    }
    SmartMotorControllerConfig &WithVoltageCompensation(units::volt_t voltageCompensation)
    {
        this->voltageCompensation = voltageCompensation;
        return *this;
    }

    SmartMotorControllerConfig &WithFollowers(std::vector<std::pair<void *, bool>> followers)
    {
        this->followers = followers;
        return *this;
    }

    void ClearFollowers()
    {
        followers = {};
    }

    SmartMotorControllerConfig &WithStatorCurrentLimit(units::ampere_t stallCurrent)
    {
        this->statorStallCurrentLimit = stallCurrent;
        return *this;
    }
    SmartMotorControllerConfig &WithSupplyCurrentLimit(units::ampere_t stallCurrent)
    {
        this->supplyStallCurrentLimit = stallCurrent;
        return *this;
    }
    SmartMotorControllerConfig &WithClosedLoopRampRate(units::second_t rate)
    {
        this->closeLoopRampRate = rate;
        return this;
    }
    SmartMotorControllerConfig &WithOpenLoopRampRate(units::second_t rate)
    {
        this->openLoopRampRate = rate;
        return this;
    }
    SmartMotorControllerConfig &WithExternalEncoder(void *externalEncoder)
    {
        this->externalEncoder = externalEncoder;
    }
    SmartMotorControllerConfig &WithGearing(MechanismGearing gearing)
    {
        this->externalEncoderGearing = gearing;
        return *this;
    }
    SmartMotorControllerConfig &WithMechanismCircumference(units::meter_t circumference)
    {
        this->mechanismCircumference = circumference;
        return *this;
    }
    SmartMotorControllerConfig &WithPeriod(units::second_t period)
    {
        this->controlPeriod = period;
        return *this;
    }

    SmartMotorControllerConfig &WithFeedforward(frc::ArmFeedforward *armFeedforward)
    {
        if (armFeedforward == nullptr)
            this->armFeedforward = {};
        else
        {
            elevatorFeedforward = {};
            simpleFeedforward = {};
            this->armFeedforward = armFeedforward;
        }
        return *this;
    }
    SmartMotorControllerConfig &WithFeedforward(frc::ElevatorFeedforward *elevatorFeedforward)
    {
        if (elevatorFeedforward == nullptr)
            this->elevatorFeedforward = {};
        else
        {
            armFeedforward = {};
            simpleFeedforward = {};
            this->elevatorFeedforward = elevatorFeedforward;
        }
        return *this;
    }
    SmartMotorControllerConfig &WithClosedLoopController(frc::ProfiledPIDController<Distance> *controller)
    {
        this->controller = (controller == nullptr) ? std::nullopt : std::make_optional(controller);
        simpleController = {};
        return *this;
    }
    SmartMotorControllerConfig &WithClosedLoopController(double kP, double kI, double kD)
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        this->controller = {};
        this->simpleController = frc::PIDController(kP, kI, kD);
    }
    SmartMotorControllerConfig &WithClosedLoopController(frc::PIDController *controller)
    {
        this->simpleController = (controller == nullptr) ? std::nullopt : std::make_optional(controller);
        this->controller = {};
        return *this;
    }
    SmartMotorControllerConfig &WithClosedLoopController(double kP, double kI, double kD, Velocity_t maxVelocity, Acceleration_t maxAcceleration)
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        this->simpleController = {};
        this->controller = frc::ProfiledPIDController<Distance>(kP, kI, kD, Constraints<Distance>(maxVelocity, maxAcceleration));
        return *this;
    }
    SmartMotorControllerConfig &WithSimpleFeedforward(frc::SimpleMotorFeedforward<Distance> *simpleFeedforward)
    {
        if (simpleFeedforward == nullptr)
            this->simpleFeedforward = {};
        else
        {
            armFeedforward = {};
            elevatorFeedforward = {};
            this->simpleFeedforward = simpleFeedforward;
        }
        return *this;
    }
    SmartMotorControllerConfig &WithExternalGearing(MechanismGearing externalEncoderGearing){
        this->externalEncoderGearing = externalEncoderGearing;
        return *this;
    }

    // Getters
    std::optional<units::ampere_t> GetStatorStallCurrentLimit() const
    {
        return statorStallCurrentLimit;
    }
    std::optional<units::ampere_t> GetSupplyStallCurrentLimit() const
    {
        return supplyStallCurrentLimit;
    }
    std::optional<units::volt_t> GetVoltageCompensation() const
    {
        return voltageCompensation;
    }
    std::optional<MotorMode> GetIdleMode() const
    {
        return idleMode;
    }
    std::optional<units::radian_t> GetMechanismLowerLimit() const
    {
        return mechanismLowerLimit;
    }
    std::optional<units::radian_t> GetMechanismUpperLimit() const
    {
        return mechanismUpperLimit;
    }
    std::optional<std::vector<std::pair<void *, bool>>> GetFollowers() const
    {
        return followers;
    }
    std::optional<units::meter_t> GetMechanismCircumference() const
    {
        return mechanismCircumference;
    }
    std::optional<frc::ArmFeedforward *> GetArmFeedforward() const
    {
        return armFeedforward;
    }
    std::optional<frc::ElevatorFeedforward *> GetElevatorFeedforward() const
    {
        return elevatorFeedforward;
    }
    std::optional<frc::SimpleMotorFeedforward<Distance> *> GetSimpleFeedforward() const
    {
        return simpleFeedforward;
    }
    std::optional<frc::ProfiledPIDController<Distance> *> GetController() const
    {
        return controller;
    }
    std::optional<frc::PIDController *> GetSimpleController() const
    {
        return simpleController;
    }
    units::second_t GetControlPeriod() const
    {
        return controlPeriod;
    }
    MechanismGearing GetGearing(){
        return gearing;
    }
    std::optional<void *> GetExternalEncoder() const
    {
        return externalEncoder;
    }
    units::second_t GetOpenLoopRampRate() const
    {
        return openLoopRampRate;
    }
    units::second_t GetClosedLoopRampRate() const
    {
        return closeLoopRampRate;
    }
    std::optional<TelemetryVerbosity> GetTelemetryVerbosity() const
    {
        return verbosity;
    }
    std::optional<std::string> GetTelemetryName() const
    {
        return telemetryName;
    }
    frc2::Subsystem* GetSubsystem() const
    {
        return subsystem;
    }
    std::optional<units::radian_t> GetZeroOffset() const
    {
        return zeroOffset;
    }
    std::optional<units::celsius_t> GetTemperatureCutoff() const
    {
        return temperatureCutoff;
    }
    bool IsEncoderInverted() const
    {
        return encoderInverted;
    }
    bool IsMotorInverted() const
    {
        return motorInverted;
    }
    bool IsUsingExternalEncoder() const
    {
        return useExternalEncoder;
    }
    std::optional<units::radian_t> GetStartingPosition() const
    {
        return startingPosition;
    }
    std::optional<units::volt_t> GetClosedLoopControllerMaximumVoltage() const
    {
        return closedLoopControllerMaximumVoltage;
    }
    std::optional<units::radian_t> GetFeedbackSynchronizationThreshold() const
    {
        return feedbackSynchronizationThreshold;
    }
    ControlMode GetMotorControllerMode() const
    {
        return motorControllerMode;
    }
    MechanismGearing GetExternalEncoderGearing() const
    {
        return externalEncoderGearing;
    }
    std::optional<units::radian_t> GetDiscontinuityPoint() const
    {
        if(maxDiscontinuityPoint.has_value() && minDiscontinuityPoint.has_value() && minDiscontinuityPoint.value() != maxDiscontinuityPoint.value() - 1_tr){
            //TODO: SmartMotorControllerConfigurationException
        }
        return maxDiscontinuityPoint;
    }
    bool GetExternalEncoderInverted() const
    {
        return externalEncoderInverted;
    }



    units::turns_per_second_t ConvertToMechanism(units::meters_per_second_t velocity) const
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return units::turns_per_second_t{velocity / mechanismCircumference.value()};
    }

    units::turns_per_second_squared_t ConvertToMechanism(units::meters_per_second_squared_t acceleration) const
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return units::turns_per_second_squared_t{acceleration / mechanismCircumference.value()};
    }

    units::turn_t ConvertToMechanism(units::meter_t distance) const
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return units::turn_t{distance / mechanismCircumference.value()};
    }

    units::meter_t ConvertFromMechanism(units::turn_t rotations) const
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return units::meter_t{rotations * mechanismCircumference.value()};
    }

    units::meters_per_second_t ConvertFromMechanism(units::turns_per_second_t velocity) const
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return units::meters_per_second_t{velocity * mechanismCircumference.value()};
    }

    units::meters_per_second_squared_t ConvertFromMechanism(units::turns_per_second_squared_t acceleration) const
    {
        if (!mechanismCircumference.has_value())
        {
            // TODO: SmartMotorControllerConfigurationException
        }
        return units::meters_per_second_squared_t{acceleration * mechanismCircumference.value()};
    }

private:
    frc2::Subsystem *subsystem;
    std::optional<void *> externalEncoder = {};
    bool externalEncoderInverted = false;
    std::optional<std::vector<std::pair<void *, bool>>> followers;
    std::optional<frc::SimpleMotorFeedforward<Distance> *> simpleFeedforward = {};
    std::optional<frc::ArmFeedforward *> armFeedforward = {};
    std::optional<frc::ElevatorFeedforward *> elevatorFeedforward = {};
    std::optional<frc::ProfiledPIDController<Distance> *> controller = {};
    std::optional<frc::PIDController *> simpleController = {};
    MechanismGearing gearing;
    MechanismGearing externalEncoderGearing = MechanismGearing(GearBox({1.0}));
    std::optional<units::meter_t> mechanismCircumference;
    units::second_t controlPeriod = 20.0_s;
    units::second_t openLoopRampRate = 0.0_s;
    units::second_t closeLoopRampRate = 0.0_s;
    std::optional<units::ampere_t> statorStallCurrentLimit = {};
    std::optional<units::ampere_t> supplyStallCurrentLimit = {};
    std::optional<units::volt_t> voltageCompensation = {};
    std::optional<MotorMode> idleMode = {};
    std::optional<units::radian_t> mechanismLowerLimit = {};
    std::optional<units::radian_t> mechanismUpperLimit = {};
    std::optional<std::string> telemetryName = {};
    std::optional<TelemetryVerbosity> verbosity = {};
    std::optional<units::radian_t> zeroOffset = {};
    std::optional<units::celsius_t> temperatureCutoff = {};
    bool encoderInverted = false;
    bool motorInverted = false;
    bool useExternalEncoder = true;
    std::optional<units::radian_t> startingPosition = {};
    std::optional<units::volt_t> closedLoopControllerMaximumVoltage = {};
    std::optional<units::radian_t> feedbackSynchronizationThreshold = {};
    ControlMode motorControllerMode = ControlMode::CLOSED_LOOP;
    std::optional<units::radian_t> maxDiscontinuityPoint = {};
    std::optional<units::radian_t> minDiscontinuityPoint = {};
};

#endif // SMART_MOTOR_CONTROLLER_CONFIG_H