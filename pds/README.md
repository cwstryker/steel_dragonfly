# Pseudo Driver Station

As the name implies, this package module implements a fake driver station.
This module allows a timed robot to enable itself immediately after the robot code starts executing.
This module should only ever be used in non-robot applications.

### This Warning Comes at the Start of the Book

Using this module on an FRC robot is an egregious violation of game rules and the spirit of FIRST.
It creates a safety hazard for everyone at the event.  RED CARD!!!  I mean it!

**You have been warned!**

### Motivation

With the introduction of the new SystemCore there is a surplus of old RoboRIO robot controllers.
We wanted to the use a RoboRIO in a non-robot application, so I developed this code to automatically enable the robot 
without a driver station attached.

### Usage

Use this package module in a wpilib.TimedRobot implementation as shown...

```python
from red_card_pds import PseudoDriverStation

...

class MyNotARobot(wpilib.TimedRobot):

    def robotInit(self):
        self.pds = PseudoDriverStation()
        ...

    def robotPeriodic(self):
        self.pds.update()
        ...

...
```

### Installation

This module is intentionally not published on PyPI.
To install this package in your non-robot application, build the wheel file and use the following commands to install 
the wheel file.
Keep a copy of the wheel file on your workstation, and use the --find-links option anytime use run the 'robotpy sync' 
command.

```commandline
PS C:\NotARobotProject> python -m pip install path/to/dist_folder/red_card_pds-1.0.0-py3-none-any.whl
PS C:\NotARobotProject> python -m robotpy installer download path/to/dist_folder/red_card_pds-1.0.0-py3-none-any.whl
PS C:\NotARobotProject> python -m robotpy sync --find-links path/to/dist_folder
PS C:\NotARobotProject> python -m robotpy deploy
```
