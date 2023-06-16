from ..interfaces.controller import controller, TransducerStates, WorkInProgress


class FlipMaster9000(controller):
    def __init__(self, receiver, publisher, scanner, flipper) -> None:
        super().__init__(receiver, publisher, scanner, flipper)

    def ready(self) -> None:
        if self.state != TransducerStates.IDLE:
            raise WorkInProgress()

        self.publisher.push("Ready for new plate.")
        self.receiver.request_feedback()
        self.publisher.push("New plate detected.")
        
        self.state = TransducerStates.SCAN
        self.scan()
        # todo:: instead of calling first we could asyncIO queue?
        
    async def scan(self) -> None:
        if self.state != TransducerStates.SCAN:
            raise WorkInProgress()

        self.publisher.post(f"Scanning ... ")
        plate_scan = await self.scanner.trigger_scan()
        self.publisher.post(f"Scan complete: [ {plate_scan.details} ]")

        if plate_scan.needs_flip():
            self.publisher.post(f"Flip necessary")
            self.state = TransducerStates.FLIP
            self.flip()
        else:
            self.publisher.post(f"Flip NOT necessary.")
            self.state = TransducerStates.DELIVER
            self.deliver()
        
    def flip(self) -> None:
        if self.state != TransducerStates.FLIP:
            raise WorkInProgress()

        self.publisher.post(f"Flipping ... ")
        self.flipper.trigger_flip()
        self.publisher.post(f"Flip complete")

    def deliver(self) -> None:
        if self.state != TransducerStates.DELIVER:
            raise WorkInProgress()

        pos = self.scanner.scan.get_position()
        self.publisher.post(f"Deliver at: [ {pos} ]")
        self.receiver.request_feedback()
        self.publisher.post(f"Delivery complete")

        self.state = TransducerStates.IDLE
        self.ready()

