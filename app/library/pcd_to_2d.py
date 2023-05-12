# This code has been take from the professorship mechatronics at Saxion University
# It has been changed so it would work in this project
import os
import matplotlib.pyplot as plt
import cv2 as cv
import open3d as o3d
from ..modules.plate_point_cloud import PlatePointCloud

# For most point clouds, the smallest resolution is 0.4mm, which gives a
# spatial resolution of 1/0.4 = 2.5 pix/mm.
spatial_res = 2.5


def to_2d(pcd: o3d.geometry.PointCloud, filename: str):
    """
    Creates a 2D image of a point cloud and saves this to data/preprocessing_output

    :param pcd: The point cloud that we need to compare
    :param filename: The filename of the pointcloud we need to comepare
    """

    plate_pcd = PlatePointCloud.from_pcd(pcd)



    # Pixel unit margin around the image.
    margin = 15

    # Project plate to image plane perpendicular to plane
    intensity, depth, mask = plate_pcd.project(spatial_res, margin=margin)

    # Optionally plot results.
    '''
    plt.imshow(intensity[:, :, ::-1])  # plot as RGB instead of BGR
    plt.title("intensity")
    plt.show()
    cv.imwrite('intensity.png', intensity)

    plt.imshow(depth, cmap='gray')
    plt.title("depth")
    plt.show()
    cv.imwrite('depth.png', depth)
    '''

    # Plot a mask of the plate
    plt.imshow(mask, cmap='gray')
    plt.title("mask")
    plt.show()

    # Get just the name of the file
    filenameSplit = filename.split('/')
    filename = filenameSplit[len(filenameSplit) - 1]
    filename = filename[:-4]

    # Save an image of the mask
    cv.imwrite(os.path.realpath('.') + os.getenv('prep_path') + filename + '.png', mask)
