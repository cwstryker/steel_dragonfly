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
