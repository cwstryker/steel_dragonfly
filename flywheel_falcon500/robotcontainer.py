# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2
from wpilib import SmartDashboard

from commands.flywheel_commands import AutoMoveFlywheel
from subsystems.flywheel_subsystem import FlywheelSubsystem


class RobotContainer:
    """
    The robot container defines the overall structure of the robot.
    """

    def __init__(self):
        self.flywheel = FlywheelSubsystem()
        self.test_flywheel = AutoMoveFlywheel(self.flywheel)
        self.driver_controller = commands2.button.CommandXboxController(0)
        self.configureButtonBindings()

        SmartDashboard.putData("Flywheel", self.flywheel)
        SmartDashboard.putData("Test Flywheel", self.test_flywheel)

    def getAutonomousCommand(self) -> commands2.Command:
        """Returns the autonomous command"""
        return self.test_flywheel

    def moveFlywheelTo(self, position: float):
        """Command the controller to move the flywheel to the specified position."""
        self.flywheel.move_to_position(position)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Move the arm to 2 radians above horizontal when the 'A' button is pressed.
        self.driver_controller.a().onTrue(
            commands2.cmd.run(lambda: self.moveFlywheelTo(100), self.flywheel)
        )

        # Move the arm to neutral position when the 'B' button is pressed
        self.driver_controller.b().onTrue(
            commands2.cmd.run(
                lambda: self.moveFlywheelTo(0),
                self.flywheel,
            )
        )
