# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2
from commands2.typing import TTrapezoidProfileState, UseOutputFunction
import wpimath.controller
import wpimath.trajectory
from phoenix6 import controls

import constants
from subsystems.armsubsystem import ArmSubsystem


class UseArmControllerOutput(UseOutputFunction):

    def __init__(self, arm_subsystem: ArmSubsystem, *args, **kwargs):
        self._arm = arm_subsystem

    def __call__(self, output: float, setpoint: TTrapezoidProfileState) -> None:
        self.accept(output, setpoint)

    def accept(
        self, output: float, setpoint: TTrapezoidProfileState
    ) -> None:
        # Calculate the feedforward from the setpoint
        self._arm.ff_voltage = self._arm.feedforward.calculate(
            setpoint.position, setpoint.velocity
        )

        # Add the feedforward to the PID output to get the motor output
        motor_voltage = output + self._arm.ff_voltage if self._arm.isEnabled() else 0.0

        # Limit the motor voltage to the interval (-12V, 12)
        max_motor_voltage = constants.ArmConstants.kCompVolts
        self.throttle = max(min(motor_voltage, max_motor_voltage), -max_motor_voltage)

        # Set the motor voltage
        request = controls.VoltageOut(0)
        self._arm.get_talonfx().set_control(request.with_output(motor_voltage))


class TurnToAngleProfiled(commands2.ProfiledPIDCommand):

    def __init__(self, target_angle_degrees: float, arm: ArmSubsystem):
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
            arm.getMeasurement,
            target_angle_degrees,
            UseArmControllerOutput(arm),
            arm,
        )

    def isFinished(self) -> bool:
        return self.getController().atGoal()
