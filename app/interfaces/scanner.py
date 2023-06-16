from ..modules.ur5_nodes import UR5MoveNode
from ..modules.plate_point_cloud import PlatePointCloud


class scanner:
    def __init__(self) -> None:
        """
        scan is the latest scan data (PlatePointCloud obj)
        """
        self.scan = None
        self.move_node = UR5MoveNode()

        # todo:: subscribe to trigger ur5 movement

    async def trigger_scan(self) -> PlatePointCloud:
        raise Exception('unimplemented')


