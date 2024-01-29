# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2
import constants
import phoenix6
import wpilib
import wpimath.controller
import wpimath.trajectory
from wpiutil import SendableBuilder


class ArmSubsystem(commands2.Subsystem):
    """A robot arm subsystem that moves with a motion profile."""

    def __init__(self) -> None:
        super().__init__()

        self.motor = (phoenix6.hardware.TalonFX(constants.ArmConstants.kMotorPort))

        # Motor configuration parameters
        config = phoenix6.configs.TalonFXConfiguration()
        current_limit = phoenix6.configs.CurrentLimitsConfigs()
        current_limit.supply_current_limit_enable = False
        current_limit.supply_current_threshold(constants.ArmConstants.kThresholdCurrent)
        current_limit.supply_cu
        config.supplyCurrLimit.triggerThresholdTime = (
            constants.ArmConstants.kThresholdTime
        )
        config.supplyCurrLimit.currentLimit = constants.ArmConstants.kHoldCurrent
        config.voltageCompSaturation = constants.ArmConstants.kCompVolts

        # Configure the motor
        self.motor.configFactoryDefault()
        self.motor.configAllSettings(config)
        self.motor.clearStickyFaults()
        self.motor.setNeutralMode(phoenix6.NeutralMode.Brake)
        self.motor.enableVoltageCompensation(False)

        self.encoder = wpilib.Encoder(
            constants.ArmConstants.kEncoderPorts[0],
            constants.ArmConstants.kEncoderPorts[1],
        )

        self.feedforward = wpimath.controller.ArmFeedforward(
            constants.ArmConstants.kSVolts,
            constants.ArmConstants.kGVolts,
            constants.ArmConstants.kVVoltSecondPerRad,
            constants.ArmConstants.kAVoltSecondSquaredPerRad,
        )

        self.encoder.setDistancePerPulse(
            constants.ArmConstants.kEncoderDistancePerPulse
        )

        # Start arm at rest in neutral position
        self.setGoal(constants.ArmConstants.kArmOffsetRads)
        self.ff_voltage = 0.0
        self.throttle = 0.0

    def useOutput(
        self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State
    ) -> None:
        # Calculate the feedforward from the setpoint
        self.ff_voltage = self.feedforward.calculate(
            setpoint.position, setpoint.velocity
        )

        # Add the feedforward to the PID output to get the motor output
        motor_voltage = output + self.ff_voltage if self.isEnabled() else 0.0

        # Convert the control voltage to a percentage and set the motor
        self.throttle = max(
            min(motor_voltage / constants.ArmConstants.kCompVolts, 1.0), -1.0
        )
        self.motor.set(phoenix6.ControlMode.PercentOutput, self.throttle)

        # This is required to enable the Falcon 500 during simulation
        phoenix6.Unmanaged.feedEnable(100)

    def getMeasurement(self) -> float:
        return self.encoder.getDistance() + constants.ArmConstants.kArmOffsetRads

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addFloatProperty("POSITION", self.getMeasurement, lambda x: None)
        builder.addFloatProperty(
            "FEED FORWARD", lambda: self.ff_voltage, lambda x: None
        )
        builder.addFloatProperty("THROTTLE", lambda: self.throttle, lambda x: None)
        builder.addFloatProperty(
            "getMotorOutputPercent", self.motor.getMotorOutputPercent, lambda x: None
        )
