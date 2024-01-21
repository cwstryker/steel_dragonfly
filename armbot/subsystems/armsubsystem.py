# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import wpimath.controller
import wpimath.trajectory
from wpilib import SendableBuilderImpl

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

        self.motor = wpilib.PWMSparkMax(constants.ArmConstants.kMotorPort)
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
        self.motor_voltage = 0.0
        self.ff_voltage = 0.0

    def _useOutput(
        self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State
    ) -> None:
        # Calculate the feedforward from the setpoint
        self.ff_voltage = self.feedforward.calculate(
            setpoint.position, setpoint.velocity
        )
        self.output = output

        # Add the feedforward to the PID output to get the motor output
        self.motor_voltage = output + self.ff_voltage if self.isEnabled() else 0.0
        self.motor.setVoltage(self.motor_voltage)

    def _getMeasurement(self) -> float:
        return self.encoder.getDistance() + constants.ArmConstants.kArmOffsetRads

    def initSendable(self, builder: SendableBuilderImpl) -> None:
        builder.addFloatProperty("Position", self._getMeasurement, lambda x: None)
        builder.addFloatProperty("Feedforward", lambda: self.ff_voltage, lambda x: None)
        builder.addFloatProperty(
            "Motor Voltage", lambda: self.motor_voltage, lambda x: None
        )
