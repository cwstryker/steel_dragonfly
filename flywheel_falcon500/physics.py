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

        # The arm gearbox represents a gearbox containing one Falcon 500 motor.
        self.gearbox = wpimath.system.plant.DCMotor.falcon500(1)
        self.flywheel_sim = wpilib.simulation.DCMotorSim(self.gearbox, GEAR_RATIO, MOI)

        self.falcon_sim = robot.container.flywheel.talonfx.sim_state
        self.falcon_sim.set_supply_voltage(11.0)

        # # Create a Mechanism2d display of an Arm
        # self.mech2d = wpilib.Mechanism2d(60, 60)
        # self.armBase = self.mech2d.getRoot("ArmBase", 30, 30)
        # self.armTower = self.armBase.appendLigament(
        #     "Arm Tower", 30, -90, 6, wpilib.Color8Bit(wpilib.Color.kBlue)
        # )
        # self.arm = self.armBase.appendLigament(
        #     "Arm", 30, self.armSim.getAngle(), 6, wpilib.Color8Bit(wpilib.Color.kYellow)
        # )
        #
        # # Put Mechanism to SmartDashboard
        # wpilib.SmartDashboard.putData("Arm Sim", self.mech2d)

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
        voltage = self.falcon_sim.motor_voltage
        self.flywheel_sim.setInputVoltage(voltage)

        # Next, we update it
        self.flywheel_sim.update(tm_diff)

        # Finally, we set our simulated encoder's readings (convert from radians to rotations)
        position = self.flywheel_sim.getAngularPosition() / (2 * math.pi)
        self.falcon_sim.set_raw_rotor_position(position)

        velocity = self.flywheel_sim.getAngularVelocity() / (2 * math.pi)
        self.falcon_sim.set_rotor_velocity(velocity)

        # # Update the mechanism arm angle based on the simulated arm angle
        # # -> setAngle takes degrees, getAngle returns radians... >_>
        # self.arm.setAngle(math.degrees(self.armSim.getAngle()))
