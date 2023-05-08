from enum import Enum

class State(Enum):
    IDLE = 1
    SCAN = 2
    FLIP = 3
    DELIVER = 4

class WorkInProgress(Exception):
    pass

class FlipMaster(object):
    def __init__(self):
        self.state = State.IDLE

    def flip(self):
        if self.state != State.IDLE:
            raise WorkInProgress()
        self.state = State.FLIP

        # todo:: send ros message to UR5 to flip + callback function from piston sensor confirmation

    def scan(self):
        if self.state != State.IDLE:
            raise WorkInProgress()
        self.state = State.SCAN

        # todo:: send ros message to UR5 to scan + callback function confirmation from FTP server

    def deliver(self):
        if self.state != State.IDLE:
            raise WorkInProgress()
        self.state = State.DELIVER

        # todo:: send ros message with position + callback function confirmation from pick up robot (ROS reply)





