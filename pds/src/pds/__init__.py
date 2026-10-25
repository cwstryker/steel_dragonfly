import socket
import wpilib


MSG = \
"""!!!!!!!!!!!!!     ATTENTION     !!!!!!!!!!!!!

  This robot controller is running code that can 
  enable it without a driver station attached.
  
   *** DO NOT RUN THIS CODE ON AN FRC ROBOT ***
  
"""


class PseudoDriverStation:
    """DO NOT USE THIS CLASS ON A COMPETITION ROBOT"""
    def __init__(self):
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        wpilib.reportError(MSG, printTrace=False)

    def update(self) -> None:
        """Instruct the controller to enable in teleop mode"""
        packet = bytearray([0x57, self.seq, 0x01, 0x04, 0x10, 0x00])
        self.sock.sendto(packet, ("127.0.0.1", 1110))
        self.seq = (self.seq + 1) & 0xFF
