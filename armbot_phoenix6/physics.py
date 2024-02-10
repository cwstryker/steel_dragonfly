#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.


import math
import typing

import wpilib
import wpilib.simulation
import wpimath.system.plant
from phoenix6.unmanaged import feed_enable
from pyfrc.physics.core import PhysicsInterface

import constants

if typing.TYPE_CHECKING:
    from robot import MyRobot


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
            constants.ArmConstants.kGearRatio,
            wpilib.simulation.SingleJointedArmSim.estimateMOI(0.762, 5),
            0.762,
            math.radians(-75),
            math.radians(255),
            True,
            math.radians(0),
        )

        # This is a simulation of the Falcon 500 motor controller.
        self.falconSim = robot.container.robot_arm._talonfx.sim_state

        # Set the battery voltage at the motor controller.
        self.falconSim.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())

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

        # This is required to keep the motor controller enabled during the simulation
        feed_enable(100)  # keep the device enabled for the next 100 ms

        # First, we set our "inputs" (voltages)
        self.armSim.setInput(0, self.falconSim.motor_voltage)

        # Next, we update the simulation state
        self.armSim.update(tm_diff)

        # Finally, we set our simulated encoder's readings (convert from radians to rotations)
        position = (
            constants.ArmConstants.kGearRatio * self.armSim.getAngle() / (2 * math.pi)
        )
        self.falconSim.set_raw_rotor_position(position)

        velocity = (
            constants.ArmConstants.kGearRatio
            * self.armSim.getVelocity()
            / (2 * math.pi)
        )
        self.falconSim.set_rotor_velocity(velocity)

        # Update the mechanism arm angle based on the simulated arm angle
        # -> setAngle takes degrees, getAngle returns radians... >_>
        self.arm.setAngle(self.armSim.getAngleDegrees())
