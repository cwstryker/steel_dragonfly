# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2

from subsystems.flywheel_subsystem import FlywheelSubsystem


class TestMoveFlywheel(commands2.Command):
    def __init__(self, flywheel: FlywheelSubsystem) -> None:
        super().__init__()
        self._flywheel = flywheel
        self.addRequirements(flywheel)
        self.finished = False

    def initialize(self) -> None:
        self._flywheel.move_to_position(50)
        self.finished = False

    def execute(self) -> None:
        if self._flywheel.median_velocity == 0:
            self.finished = True

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.finished = True

    def isFinished(self) -> bool:
        return self.finished
