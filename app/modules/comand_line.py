from ..interfaces.publisher import publisher
from ..interfaces.receiver import reciever

class CommandLinePublisher(publisher):
    def __init__(self) -> None:
        super().__init__()

    def push(self, msg):
        print(msg)


class CommandLineReceiver(reciever):
    def __init__(self) -> None:
        super().__init__()

    def request_feedback():
        input = None
        while input != '\n':
            input = input('> ')

