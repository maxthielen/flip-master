import os
import rclpy
from rclpy.node import Node
from srv import AddTwoInts
from time import sleep
from skimage import io
from app.library.pcd_to_2d import spatial_res
from app.modules.point_cloud import PointCloud

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

def set_global():
    os.environ['img_path'] = "/data/img/"
    os.environ['pcd_path'] = "/data/pcd/"
    os.environ['prep_path'] = "/data/prep/"
    os.environ['mm_per_dist'] = "1"


if __name__ == '__main__':
    set_global()

    rclpy.init()
    node = MinimalService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

    # pc = PointCloud('03', '-bin.pcd')
    # pc.show()
    # features = pc.extract_features()
    # holes = pc.segment_image(io.imread(os.path.realpath('.') + os.getenv('prep_path') + '03-bin.png'), 1/spatial_res)
    
    # print(f"Features: {features}")
    # print(f"Holes: {holes}")