import inspect

import wpilib
from red_card_pds import PseudoDriverStation


class MyNotARobot(wpilib.TimedRobot):
    """
    ##########################################################################
    Example Program
    This program is an example of how to use the PseudoDriverStation class.
    """

    # noinspection PyAttributeOutsideInit
    def robotInit(self):
        """
        This function is called upon program startup and should be used for any
        initialization code.
        """
        # Display information about this robot program on the driver station console
        print(inspect.getdoc(self))

        # initialize the robot
        self.pds = PseudoDriverStation()
        ...

    def robotPeriodic(self):
        """This function is called periodically regardless of the robot's state"""
        self.pds.update()
        ...

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        ...

    def teleopPeriodic(self):
        """This function is called periodically during teleop mode."""
        ...

    def teleopExit(self):
        """This function is called when teleop mode ends."""
        ...

    def disabledPeriodic(self):
        """This function is called periodically when the robot is disabled"""
        ...

