import os
from time import sleep
from skimage import io
from lib.pcd_to_2d import spatial_res
from src.point_cloud import PointCloud

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'Hello World: %d' % self.i
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


def set_global():
    os.environ['img_path'] = "/data/img/"
    os.environ['pcd_path'] = "/data/pcd/"
    os.environ['prep_path'] = "/data/prep/"
    os.environ['mm_per_dist'] = "1"


if __name__ == '__main__':
    set_global()

    while True:
        print("Hello World!")
        print(os.environ['img_path'])
        sleep(10)

    # pc = PointCloud('03', '-bin.pcd')
    # pc.show()
    # features = pc.extract_features()
    # holes = pc.segment_image(io.imread(os.path.realpath('.') + os.getenv('prep_path') + '03-bin.png'), 1/spatial_res)
    
    # print(f"Features: {features}")
    # print(f"Holes: {holes}")