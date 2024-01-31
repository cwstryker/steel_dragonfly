# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpimath.filter
from commands2 import Subsystem
from phoenix6 import configs, controls, hardware
from wpiutil import SendableBuilder


class FlywheelSubsystem(Subsystem):
    """
    A simple flywheel driven by a Falcon 500 motor.  Motion Magic is used to
    move the flywheel to the requested position using velocity control.
    """

    def __init__(self):
        super().__init__()

        # Configure the motor controller
        self.talonfx = hardware.TalonFX(0)
        talonfx_configs = configs.TalonFXConfiguration()

        # Set current limits to prevent brown out of the motor controller
        current_limits = talonfx_configs.current_limits
        current_limits.supply_current_limit = 30
        current_limits.supply_current_limit_enable = True

        # Set slot 0 gains
        slot0_configs = talonfx_configs.slot0
        slot0_configs.k_s = 0.25  # Add 0.25 V output to overcome static friction
        slot0_configs.k_v = 0.12  # A velocity target of 1 rps results in 0.12 V output
        slot0_configs.k_a = 0.01  # An acceleration of 1 rps/s requires 0.01 V output
        slot0_configs.k_p = (
            4.8  # A position error of 2.5 rotations results in 12 V output
        )
        slot0_configs.k_i = 0  # no output for integrated error
        slot0_configs.k_d = 0.1  # A velocity error of 1 rps results in 0.1 V output

        # Set Motion Magic settings
        motion_magic_configs = talonfx_configs.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = (
            80  # Target cruise velocity of 80 rps
        )
        motion_magic_configs.motion_magic_acceleration = (
            160  # Target acceleration of 160 rps/s (0.5 seconds)
        )
        motion_magic_configs.motion_magic_jerk = (
            1600  # Target jerk of 1600 rps/s/s (0.1 seconds)
        )

        # Apply the motor configuration
        self.talonfx.configurator.apply(talonfx_configs)

        # Create a Motion Magic request, voltage output
        self.request = controls.MotionMagicVoltage(0)

        # Create a simulation object
        self.sim_state = self.talonfx.sim_state

    def initSendable(self, builder: SendableBuilder) -> None:
        """Setup the SmartDashboard"""
        builder.addFloatProperty(
            "position",
            lambda: float(self.talonfx.get_position().value),
            lambda value: None,
        )
        builder.addFloatProperty(
            "velocity",
            lambda: float(self.talonfx.get_velocity().value),
            lambda value: None,
        )
        builder.addStringProperty(
            "Enabled", lambda: str(self.talonfx.get_device_enable()), lambda value: None
        )

    def move_to_position(self, position: float):
        """Move the flywheel to a specific position"""
        self.talonfx.set_control(self.request.with_position(position))
