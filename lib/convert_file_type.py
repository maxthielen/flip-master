"""
This file adjusted from the Plate Matching implementation (WP2)

Authors:
    - [Lisa Molhoek](https://gitlab.com/lisamolhoek)
    - [Tiemen Wierda](https://gitlab.com/TWierda)
    - [Tom Mulder](https://gitlab.com/tommulmul)
    - [Thomas Beunders](hhttps://gitlab.com/Beunhaas)

Editor:
    - [Max Thielen](497441@student.saxion.nl)
"""

import os
from PIL import Image
import numpy as np
import cv2 as cv
# For visualization without ROS we can use open3d (matplotlib is horribly slow
# with this amount of data points). An alternative might be plotly. open3d is
# also used to save the point cloud as .pcd file format.
import open3d as o3d


def img_to_pcd(img_filename, compress=True, verbose=False):
    img_path = os.path.realpath('.') + os.getenv("img_path") + img_filename
    pil_img = Image.open(img_path)
    if pil_img is None:
        print("[img_to_pcd]: did not receive png type file")
        return ""

    # example: file_path    = IMG_PATH/pic.png
    #          output_path  = PCD-PATH/pic-bin.pcd
    pcd_path = os.path.realpath('.') + os.getenv("pcd_path") + img_filename.split('.')[0]
    if compress:
        pcd_path += "-bin.pcd"
    else:
        pcd_path += ".pcd"

    if verbose:
        print(f"({img_path}) --> [{pcd_path}]")

    # Get resolution information that is stored in the files metadata
    pil_im = Image.open(img_path)
    pil_im.load()  # needed to access metadata

    x_resolution = float(pil_im.info['Dx'])
    y_resolution = float(pil_im.info['Dy'])
    z_resolution = float(pil_im.info['Dz'])

    # Clear pillow image, not used further.
    del pil_im

    # Read image with OpenCV as grayscale
    im = cv.imread(img_path, cv.IMREAD_GRAYSCALE)

    h, w = im.shape

    # Actual height of the image is a third of the PNG resolution
    h = h // 3

    # The first third of the image is the intensity, in 8-bit
    intensity = im[:h, :]

    # The last two-thirds of the image is the height map in 16-bit, which means
    # that every value requires two pixels, and the resulting rows are twice as
    # wide as the actual rows (and therefore exceed the image width and take up
    # two row heights).
    heightmap = im[h:, :].reshape(h, -1)

    # The least-significant bits are placed first, and can be extracted with
    # the following slice, using a step size of 2.
    heightmap_lsb = heightmap[:, ::2]
    # The most-significant bits are second, and can be extracted with the same
    # slice but offset by a single pixel.
    heightmap_msb = heightmap[:, 1::2]

    # They can be combined by adding the scaled MSB to the LSB. The result is a
    # 16-bit unsigned int numpy array.
    heightmap_tot = heightmap_lsb + 256 * heightmap_msb

    # Create a meshgrid where ys = [[0, 0, 0, ...], [1, 1, 1, ...]] and
    # xs = [[1, 2, 3, ...], [1, 2, 3, ...]]. Choose 'ij' indexing because our
    # height map is an image.
    ys, xs = np.meshgrid(range(h), range(w), indexing='ij')

    # Reshape x, y, z, and values into single dimension arrays and scale by
    # their respective resolutions.
    xs2 = xs.reshape(intensity.size, -1) * x_resolution
    ys2 = ys.reshape(intensity.size, -1) * y_resolution
    zs2 = heightmap_tot.reshape(intensity.size, -1) * z_resolution

    # Optionally remap intensity from grayscale to color map, we need 3 color
    # channels anyway, so why not use them?
    intensity_color = cv.applyColorMap(intensity, cv.COLORMAP_JET)

    # Create index array to filter 0 values from point cloud later
    intensity_filter = np.where(intensity.reshape(intensity.size, -1) == 0)

    # Map colors to [0, 1] range (OpenCV uses BGR instead of RGB)
    bs = intensity_color[:, :, 0].reshape(intensity.size, -1) / 256
    gs = intensity_color[:, :, 1].reshape(intensity.size, -1) / 256
    rs = intensity_color[:, :, 2].reshape(intensity.size, -1) / 256

    vals = np.hstack((rs, gs, bs))

    # Create an (n x 3) numpy array with the points location.
    points = np.hstack((xs2, ys2, zs2))

    # Create single point cloud array
    pointcloud = np.hstack((points, vals))

    # Remove zero intensity values
    pointcloud_filtered = np.delete(pointcloud, intensity_filter, axis=0)

    # Create the point cloud object and set the values accordingly.
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_filtered[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(pointcloud_filtered[:, 3:])

    if compress:
        # Save as .pcd pointcloud, in compressed, binary output.
        o3d.io.write_point_cloud(pcd_path, pcd, compressed=True)
    else:
        # Alternatively you can use to get a text-based file (which is a lot
        # bigger, but human-readable).
        o3d.io.write_point_cloud(pcd_path, pcd, write_ascii=True)
    return pcd_path
