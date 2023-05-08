import os
from src.point_cloud import PointCloud

from skimage import io
from lib.pcd_to_2d import spatial_res


def set_global():
    os.environ['img_path'] = "/data/img/"
    os.environ['pcd_path'] = "/data/pcd/"
    os.environ['prep_path'] = "/data/prep/"
    os.environ['mm_per_dist'] = "1"


if __name__ == '__main__':
    set_global()

    pc = PointCloud('03', '-bin.pcd')
    # pc.show()
    features = pc.extract_features()
    holes = pc.segment_image(io.imread(os.path.realpath('.') + os.getenv('prep_path') + '03-bin.png'), 1/spatial_res)

    print(f"Features: {features}")
    print(f"Holes: {holes}")





