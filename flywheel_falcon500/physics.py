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

if typing.TYPE_CHECKING:
    from robot import FlywheelRobot

GEAR_RATIO = 1.0
MOI = 0.00117  # kg * m^2, SDS Brass Flywheel - 4.0 lb * in^2


class PhysicsEngine:
    """
    Simulates a motor with a large MOI
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "FlywheelRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        # This is a simulation of the Falcon 500 motor connected to a gearbox.
        self.gearbox = wpimath.system.plant.DCMotor.falcon500(1)

        # This is a physics simulation of a motor connected to a flywheel.
        # The DCMotorSim is used because it tracks position of the motor.
        self.flywheel_sim = wpilib.simulation.DCMotorSim(self.gearbox, GEAR_RATIO, MOI)

        # This is a simulation of the Falcon 500 motor controller.
        self.falcon_sim = robot.container.flywheel.talonfx.sim_state

        # Set the battery voltage at the motor controller.
        self.falcon_sim.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())

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
        self.flywheel_sim.setInputVoltage(self.falcon_sim.motor_voltage)

        # Next, we update the simulation state
        self.flywheel_sim.update(tm_diff)

        # Finally, we set our simulated encoder's readings (convert from radians to rotations)
        position = self.flywheel_sim.getAngularPosition() / (2 * math.pi)
        self.falcon_sim.set_raw_rotor_position(position)

        velocity = self.flywheel_sim.getAngularVelocity() / (2 * math.pi)
        self.falcon_sim.set_rotor_velocity(velocity)
