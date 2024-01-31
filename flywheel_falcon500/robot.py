# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import typing

import commands2
import wpilib

from robotcontainer import RobotContainer


class FlywheelRobot(commands2.TimedCommandRobot):
    """
    This robot consists of a single Falcon 500 motor driving a flywheel through a
    gearbox.
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function initializes the robot control code at power up.
        """
        self.container = RobotContainer()

    def robotPeriodic(self) -> None:
        """
        This function is called every 20 ms, no matter the mode. Use this for items
        like diagnostics that you want ran during disabled, autonomous, teleoperated
        and test. This runs after the mode specific periodic functions, but before
        LiveWindow and SmartDashboard integrated updating.
        """
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when the robot is in disabled mode."""
        pass

    def autonomousInit(self) -> None:
        """This function is called once when the robot enters autonomous mode."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically when the robot is in autonomous mode."""
        pass

    def teleopInit(self) -> None:
        """This function is called periodically when the robot is in teleop mode."""

        # This makes sure that the autonomous stops running when teleop starts running.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(FlywheelRobot)
