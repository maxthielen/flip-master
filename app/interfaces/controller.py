import enum


class WorkInProgress(Exception):
    pass


class TransducerStates(enum.Enum):
    IDLE = 'idle'
    SCAN = 'scan'
    FLIP = 'flip'
    DELIVER = 'deliver'


class controller:
    def __init__(self, receiver, publisher, scanner, flipper) -> None:
        self.reveiver = receiver
        self.publisher = publisher
        self.scanner = scanner
        self.flipper = flipper
        self.state = TransducerStates.IDLE

    
