# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2


class RobotContainer:
    """
    The robot container defines the overall structure of the robot.
    """

    def __init__(self):
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        """Returns the autonomous command"""
        return commands2.cmd.none()
