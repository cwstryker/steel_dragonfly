# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math

import commands2
import wpimath.controller
import wpimath.trajectory
from phoenix6 import configs, controls, hardware, signals, unmanaged
from wpiutil import SendableBuilder

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

        self._talonfx = hardware.TalonFX(constants.ArmConstants.kMotorPort)

        # Setup the motor configuration
        talonfx_configs = configs.TalonFXConfiguration()
        talonfx_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE

        talonfx_currents = talonfx_configs.current_limits
        talonfx_currents.supply_current_limit_enable = False
        talonfx_currents.supply_current_threshold = (
            constants.ArmConstants.kThresholdCurrent
        )
        talonfx_currents.supply_time_threshold = constants.ArmConstants.kThresholdTime
        talonfx_currents.supply_current_limit = constants.ArmConstants.kHoldCurrent

        # Apply the motor configuration
        self._talonfx.configurator.apply(talonfx_configs)
        self._talonfx.configurator.clear_sticky_faults()

        # Create a simulation object
        self.sim_state = self._talonfx.sim_state

        # Create a feed forward voltage model
        self.feedforward = wpimath.controller.ArmFeedforward(
            constants.ArmConstants.kSVolts,
            constants.ArmConstants.kGVolts,
            constants.ArmConstants.kVVoltSecondPerRad,
            constants.ArmConstants.kAVoltSecondSquaredPerRad,
        )

        # Start arm at rest in neutral position
        self._talonfx.set_position(0)
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

        # Limit the motor voltage to the interval (-12V, 12)
        max_motor_voltage = constants.ArmConstants.kCompVolts
        self.throttle = max(min(motor_voltage, max_motor_voltage), -max_motor_voltage)

        # Set the motor voltage
        request = controls.VoltageOut(0)
        self._talonfx.set_control(request.with_output(motor_voltage))

    def getMeasurement(self) -> float:
        """Returns the position of the arm in radians"""
        rotations = self._talonfx.get_position().value
        return rotations / constants.ArmConstants.kGearRatio * (2 * math.pi)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addFloatProperty(
            "ARM POSITION (radians)", self.getMeasurement, lambda x: None
        )
        builder.addFloatProperty(
            "FEED FORWARD (volts)", lambda: self.ff_voltage, lambda x: None
        )
        builder.addFloatProperty("THROTTLE", lambda: self.throttle, lambda x: None)
        builder.addFloatProperty(
            "MOTOR VOLTAGE (volts)",
            lambda: self._talonfx.get_motor_voltage().value,
            lambda x: None,
        )
        builder.addFloatProperty(
            "MOTOR POSITION (rotations)",
            lambda: float(self._talonfx.get_position().value),
            lambda value: None,
        )

    def get_talonfx(self):
        return self._talonfx
