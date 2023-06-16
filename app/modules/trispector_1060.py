from ..interfaces.scanner import scanner
from .plate_point_cloud import PlatePointCloud


class Trispector1060(scanner):
    def __init__(self) -> None:
        super().__init__()

        # subscribe to scanner ros node

    def trigger_scan(self) -> PlatePointCloud:
        self.scan = self.move_node.trigger_sweep_scan()
        # todo:: prossess the scan into the plate point cloud
        return self.scan
        

