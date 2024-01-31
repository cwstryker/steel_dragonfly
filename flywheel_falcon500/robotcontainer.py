# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2
from wpilib import SmartDashboard

from commands.flywheel_commands import TestMoveFlywheel
from subsystems.flywheel_subsystem import FlywheelSubsystem


class RobotContainer:
    """
    The robot container defines the overall structure of the robot.
    """

    def __init__(self):
        self.flywheel = FlywheelSubsystem()
        self.test_flywheel = TestMoveFlywheel(self.flywheel)
        SmartDashboard.putData("Flywheel", self.flywheel)
        SmartDashboard.putData("Test Flywheel", self.test_flywheel)

    def getAutonomousCommand(self) -> commands2.Command:
        """Returns the autonomous command"""
        return self.test_flywheel
