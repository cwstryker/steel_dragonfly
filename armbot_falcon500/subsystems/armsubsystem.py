# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import wpimath.controller
import wpimath.trajectory
from wpilib import SendableBuilderImpl
import phoenix5

import constants


class ArmSubsystem(commands2.ProfiledPIDSubsystem):
    """A robot arm subsystem that moves with a motion profile."""

    # Create a new ArmSubsystem
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.ArmConstants.kP,
                0,
                0,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    constants.ArmConstants.kMaxVelocityRadPerSecond,
                    constants.ArmConstants.kMaxAccelerationRadPerSecSquared,
                ),
            ),
            0,
        )

        self.motor = phoenix5.TalonFX(constants.ArmConstants.kMotorPort)

        # Motor configuration parameters
        config = phoenix5.TalonFXConfiguration()
        config.supplyCurrLimit.enable = False
        config.supplyCurrLimit.triggerThresholdCurrent = (
            constants.ArmConstants.kThresholdCurrent
        )
        config.supplyCurrLimit.triggerThresholdTime = (
            constants.ArmConstants.kThresholdTime
        )
        config.supplyCurrLimit.currentLimit = constants.ArmConstants.kHoldCurrent
        config.voltageCompSaturation = constants.ArmConstants.kCompVolts

        # Configure the motor
        self.motor.configFactoryDefault()
        self.motor.configAllSettings(config)
        self.motor.clearStickyFaults()
        self.motor.setNeutralMode(phoenix5.NeutralMode.Brake)
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

    def _useOutput(
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
        self.motor.set(phoenix5.ControlMode.PercentOutput, self.throttle)

        # This is required to enable the Falcon 500 during simulation
        phoenix5.Unmanaged.feedEnable(100)

    def _getMeasurement(self) -> float:
        return self.encoder.getDistance() + constants.ArmConstants.kArmOffsetRads

    def initSendable(self, builder: SendableBuilderImpl) -> None:
        builder.addFloatProperty("POSITION", self._getMeasurement, lambda x: None)
        builder.addFloatProperty(
            "FEED FORWARD", lambda: self.ff_voltage, lambda x: None
        )
        builder.addFloatProperty("THROTTLE", lambda: self.throttle, lambda x: None)
        builder.addFloatProperty(
            "getMotorOutputPercent", self.motor.getMotorOutputPercent, lambda x: None
        )
