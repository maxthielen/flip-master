import os
from skimage import io

from app.library.pcd_to_2d import spatial_res
from app.modules.point_cloud import PointCloud

# from app.modules.flip_master_9000 import FlipMaster9000
# from app.modules.flippo import Flippo
# from app.modules.trispector_1060 import Trispector1060
# from app.modules.comand_line import CommandLinePublisher, CommandLineReceiver
# # from app.modules.ur5_nodes import UR5IONode, UR5MoveNode

def set_global():
    os.environ['img_path'] = "/data/img/"
    os.environ['pcd_path'] = "/data/pcd/"
    os.environ['prep_path'] = "/data/prep/"
    os.environ['mm_per_dist'] = "1"

def test_point_cloud_processing():
    pc = PointCloud('03', '-bin.pcd')
    pc.show()
    features = pc.extract_features()
    print(f"Features: {features}")

    holes = pc.segment_image(io.imread(os.path.realpath('.') + os.getenv('prep_path') + '03-bin.png'), 1/spatial_res)
    print(f"Holes: {holes}")

# def main():
#     rec = CommandLineReceiver()
#     pub = CommandLinePublisher()
    
#     move_it_node = UR5MoveNode()
#     io_node = UR5IONode()
#     tri = Trispector1060(move_it_node)
#     flip = Flippo(io_node)
    
#     FlipMaster9000(rec, pub, tri, flip)


if __name__ == '__main__':
    set_global()

    # main()
    test_point_cloud_processing()

    # import open3d
    # import numpy as np

    # pcd = open3d.geometry.PointCloud()
    # np_points = np.random.rand(100000, 3)

    # # From numpy to Open3D
    # pcd.points = open3d.utility.Vector3dVector(np_points)

    # # From Open3D to numpy
    # np_points = np.asarray(pcd.points)

    # open3d.visualization.draw_geometries([pcd])