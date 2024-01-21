#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib
import wpilib.simulation

import wpimath.system.plant
from pyfrc.physics.core import PhysicsInterface

import math
import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot

GEAR_RATIO = 300


class PhysicsEngine:
    """
    Simulates a single joint robot with joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        # The arm gearbox represents a gearbox containing one Falcon 500 motor.
        self.armGearbox = wpimath.system.plant.DCMotor.falcon500(1)

        # Simulation classes help us simulate what's going on, including gravity.
        # This sim represents an arm with 1 Falcon 500, a 300:1 reduction, a mass of
        # 5kg, 30in overall arm length, range of motion in [-75, 255] degrees, and noise
        # with a standard deviation of 1 encoder tick.
        self.armSim = wpilib.simulation.SingleJointedArmSim(
            self.armGearbox,
            GEAR_RATIO,
            wpilib.simulation.SingleJointedArmSim.estimateMOI(0.762, 5),
            0.762,
            math.radians(-75),
            math.radians(255),
            True,
            math.radians(0),
        )
        self.encoderSim = wpilib.simulation.EncoderSim(
            robot.container.robot_arm.encoder
        )
        self.motorSim = robot.container.robot_arm.motor.getSimCollection()
        self.motorSim.setBusVoltage(11.0)

        # Create a Mechanism2d display of an Arm
        self.mech2d = wpilib.Mechanism2d(60, 60)
        self.armBase = self.mech2d.getRoot("ArmBase", 30, 30)
        self.armTower = self.armBase.appendLigament(
            "Arm Tower", 30, -90, 6, wpilib.Color8Bit(wpilib.Color.kBlue)
        )
        self.arm = self.armBase.appendLigament(
            "Arm", 30, self.armSim.getAngle(), 6, wpilib.Color8Bit(wpilib.Color.kYellow)
        )

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("Arm Sim", self.mech2d)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # First, we set our "inputs" (voltages)
        self.armSim.setInput(0, self.motorSim.getMotorOutputLeadVoltage())

        # Next, we update it
        self.armSim.update(tm_diff)

        # Finally, we set our simulated encoder's readings
        self.encoderSim.setDistance(self.armSim.getAngle())

        # Update the mechanism arm angle based on the simulated arm angle
        # -> setAngle takes degrees, getAngle returns radians... >_>
        self.arm.setAngle(math.degrees(self.armSim.getAngle()))
