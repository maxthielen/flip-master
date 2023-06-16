# %%
# This code has been take from the professorship mechatronics at Saxion University
# It has been changed so it would work in this project
import open3d as o3d
import numpy as np
import cv2 as cv
# taken from
# https://robotics-foundation.readthedocs.io/en/latest/python/num.html
from ..library import pcd_utils as mr


class PlatePointCloud(object):

    def __init__(self, point_cloud, rectify=True) -> None:
        self.raw_pc = point_cloud

        # The distance_threshold has been changed so to plates would be more visible
        bg_plane, bg_inliers = self.raw_pc.segment_plane(distance_threshold=0.01,
                                                         ransac_n=3,
                                                         num_iterations=1000)

        self.bg_plane = bg_plane
        self.bg_pc = self.raw_pc.select_by_index(bg_inliers)

        remainder_pc = self.raw_pc.select_by_index(bg_inliers, invert=True)

        # The distance_threshold has been changed so to plates would be more visible
        plane, inliers = remainder_pc.segment_plane(distance_threshold=2,
                                                    ransac_n=3,
                                                    num_iterations=1000)

        self.plane = plane
        self.pc = remainder_pc.select_by_index(inliers)

        # Remove everything not part of the background plane or plate plane.
        del remainder_pc

        # Calculate some properties
        a1, b1, c1, d1 = self.bg_plane
        _, _, _, d2 = self.plane

        self.thickness = abs(d2 - d1)/(a1**2 + b1**2 + c1**2)**0.5

        unit_normal_vec1 = self.bg_plane[:3] / \
            np.linalg.norm(self.bg_plane[:3])
        unit_normal_vec2 = self.plane[:3] / \
            np.linalg.norm(self.plane[:3])

        self.angle_between_plate_and_background = \
            np.arccos(np.dot(unit_normal_vec1, unit_normal_vec2))

        # The correct width and height are only calculated after rectifying
        self.dimensions = (None, None)
        self.is_rectified = False

        if rectify:
            self.rectify()

    def print_plane(self):
        self._print_plane(self.plane)

    def print_bg_plane(self):
        self._print_plane(self.bg_plane)

    def rectify(self):
        if self.is_rectified:
            print('Already rectified')
            return None

        # Get the plate plane normal vector as unit vector
        plate_normal_vector = self.plane[:3] / np.linalg.norm(self.plane[:3])

        # If the normal vector points down from the plane, flip it. We want to
        # look from 'above'.
        if plate_normal_vector[2] < 0:
            plate_normal_vector *= -1

        # Calculate the cross product of the plate plane normal vector and the
        # world frame z-axis. The direction of the cross product gives the
        # vector to rotate around, and its magnitude gives the angle to rotate
        # with.
        angular_velocity_vector = np.cross(np.array([[0, 0, 1]]),
                                           plate_normal_vector)

        # The following function gives us the rotation matrix to achieve the
        # rotation given by the vector found above. It separates direction and
        # magnitude for us.
        R = mr.vec_to_SO3(-angular_velocity_vector)

        # Get point cloud points in numpy array.
        xyz = np.asarray(self.pc.points)

        # Rotate to get plate plane aligned with xy-plane.
        xyz_rotated = xyz @ R.T

        # Optionally check standard deviation of un-rotated and rotated
        # z-coordinates to confirm that the rotated ones are more closely
        # distributed.
        # print('Un-rotated: {}, Rotated: {}'.format(
        #    np.std(xyz[:, 2]), np.std(xyz_rotated[:, 2])))

        # Omit z-coordinates, as theoretically they are all the same. Then we
        # can use the 2D functions from OpenCV for calculating the rotated
        # bounding box.
        xy = xyz_rotated[:, :2].astype(np.float32)
        center, (width, height), angle = cv.minAreaRect(xy)
        angle = np.deg2rad(angle)
        z = (xyz_rotated[:, 2].min() + xyz_rotated[:, 2].max()) / 2
        center = np.array([[center[0], center[1], z]]).T

        # We want the resulting image to be wider than it is high, so if that
        # is not the case, we rotate by another 90 degrees (or pi/2).
        if height > width:
            height, width = width, height
            angle += np.pi/2

        # Now we have the right dimensions for width and height.
        self.dimensions = (width, height)

        # Transform the point cloud such that the center is in the origin and
        # the plates dimensions are aligned with the x and y axes.
        R2 = mr.vec_to_SO3(np.array([[0, 0, 1]]).T, angle)
        T_b0 = mr.R_p_to_SE3(R2, center)
        T_0b = mr.inv_SE3(T_b0) @ mr.R_p_to_SE3(R, np.zeros((3, 1)))

        self.pc.transform(T_0b)

        # Transform the plane with the same transformation matrix (see also
        # https://stackoverflow.com/a/48408289/4685949)
        self.plane = self.plane @ T_b0
        # And normalize (http://www.songho.ca/math/plane/plane.html)
        self.plane /= np.linalg.norm(self.plane[:3])

        # Do the same for the background point cloud and plane.
        self.bg_pc.transform(T_0b)
        self.bg_plane = self.bg_plane @ T_b0
        self.bg_plane /= np.linalg.norm(self.bg_plane[:3])

        self.is_rectified = True

    def project(self, spatial_res, margin=15):
        """
        Project the point cloud to an image, using an ideal projection matrix.

        Args:
            spatial_res: Spatial resolution in pix/mm, used to determine the
                         pixel size.
            margin:      Width of empty pixels around the returned images.

        Returns:
            Tuple of images with 1. intensity, 2. depth, and 3. binary mask.
        """

        # Set the image size that just fits the plate and the margin (on both
        # sides). Image size is typically denoted in (rows, columns), while the
        # plate dimensions are denoted as (x size, y size), therefore we have
        # to invert the 'self.dimensions' tuple.
        im_size = [np.int32(np.ceil(dim * spatial_res) + 2 * margin) for dim
                   in self.dimensions[::-1]]

        # Focal length is chosen to fit the spatial resolution. The 1000 is
        # scaling from meter to millimeter, and the 1 is the distance from the
        # camera center to the point cloud's center. The focal length is
        # expressed in [-] * [mm] * [pix/mm] = [pix] units.
        focal_length = 1000 * 1 * spatial_res

        # Build camera intrinsic matrix.
        intrinsics = np.array([[focal_length, 0, im_size[1]/2, 0],
                               [0, focal_length, im_size[0]/2, 0],
                               [0, 0, 1, 0]])

        # Rotation matrix for camera looking down the z-axis of the point cloud
        # frame.
        R = np.array([[1, 0, 0],
                     [0, -1, 0],
                     [0, 0, -1]])

        # Translate camera to look from above.
        p = np.array([[0, 0, 1]]).T

        # Create transformation matrix from R and p, and invert it to get the
        # camera extrinsics.
        T_0c = mr.R_p_to_SE3(R, p)
        T_c0 = mr.inv_SE3(T_0c)
        extrinsics = T_c0

        # Construct the ideal projection matrix.
        projection_matrix = intrinsics @ extrinsics

        # Convert the point cloud from millimeters to meters, transpose for
        # easy transformation, and convert to homogeneous form.
        pts_0 = np.asarray(self.pc.points).T / 1000
        Pts_0 = self._homogeneous(pts_0)

        # Use the projection matrix to project 3D world frame point cloud
        # points into 2D (homogeneous) image points, and convert to
        # non-homogeneous form.
        Pts_i = projection_matrix @ Pts_0
        pts_i = self._nonhomogeneous(Pts_i)

        # Optionally: show boundaries of projected image points.
        print("boundaries of projected image points:")
        print(pts_i[0, :].min(),
              pts_i[1, :].min(),
              pts_i[0, :].max(),
              pts_i[1, :].max())

        # Pre-allocate arrays for a 3-channel intensity image, 1-channel depth
        # image, and a binary mask.
        intensity = np.zeros((*im_size, 3), dtype=np.uint8)
        depth = np.empty(im_size, dtype=np.float64)
        depth.fill(np.nan)
        mask = np.zeros(im_size, dtype=np.uint8)

        for pt_i, col, pt_0 in zip(pts_i.T, np.asarray(self.pc.colors),
                                   pts_0.T):
            u = int(pt_i[0].item())
            v = int(pt_i[1].item())
            r = round(col[0].item() * 255)
            g = round(col[1].item() * 255)
            b = round(col[2].item() * 255)
            z = pt_0[2]

            # Check if the resulting point is inside the image boundaries.
            if not (v < 0 or u < 0 or v >= im_size[0] or u >= im_size[1]):
                # If so, set mask value to high.
                mask[v, u] = 255

                # If the pixel is still blank (nan), then assign the color
                # value to the intensity image, and the z-value to the depth
                # image.
                if np.isnan(depth[v, u].item()):
                    intensity[v, u] = np.array([b, g, r])
                    depth[v, u] = z
                else:
                    # If the pixel is already set, check whether the current
                    # point is 'closer to the camera' (higher in world frame).
                    # If so, overwrite the corresponding pixel.
                    if z > depth[v, u]:
                        intensity[v, u] = np.array([b, g, r])
                        depth[v, u] = z

        # The depth image is not really needed for further processing, but we
        # can still get a 'nice' image by removing all the nan values. For that
        # we first get the min and max value, and determine a new minimal value
        # to fill the nan values.
        min_depth = np.nanmin(depth)
        max_depth = np.nanmax(depth)
        # As new min we use the total range (max - min), and subtract it from
        # the current min. This gives min - (max - min) = 2 * min - max. This
        # way, we can still use half of the total data size for the range, and
        # also still easily distinguish the originally nan values.
        new_min_depth = 2 * min_depth - max_depth

        # Replace nan values with new minimum.
        depth[np.where(np.isnan(depth))] = new_min_depth

        # Then linearly scale every depth to a 0-255 range for a single channel
        # uint8 image.
        depth = (depth - new_min_depth) / (max_depth - new_min_depth) * 255

        # With the chosen spatial resolution, there are a lot of single pixel
        # holes in the mask. These can be easily mitigated by using a simple
        # morphological operation with a small kernel. See
        # https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html.
        kernel = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)

        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=1)

        return (intensity, depth, mask)

    @staticmethod
    def _homogeneous(pts: np.array) -> np.array:
        """
        Return the homogeneous form of the nonhomogeneous array of points in
        pts.
        """
        length = pts.shape[1]
        return np.vstack((pts, np.ones((1, length))))

    @staticmethod
    def _nonhomogeneous(pts: np.array) -> np.array:
        """
        Return the nonhomogeneous form of the homogeneous array of points in
        pts.
        """
        return pts[:2, :] / pts[2, :]

    @staticmethod
    def _print_plane(plane):
        print("Plane equation: {:.5f}x + {:.5f}y + {:.5f}z + {:.5f} = 0".
              format(*plane))

    @classmethod
    def from_pcd_file(cls, fn, crop=True, **kwargs):
        pc = o3d.io.read_point_cloud(str(fn))

        if crop:
            bb = fn.get_axis_aligned_bounding_box()
            bb.min_bound = (9.07113, 50, 378.36)
            bb.max_bound = (545.996, 450, 411.72)
            pc = fn.crop(bb)

        return cls(pc, **kwargs)

    @classmethod
    def from_pcd(cls, fn, **kwargs):
        """
        :return: A PlatePointCloud object.
        This has been changed from from_pcd_file because in project VISIR a crop was not needed.
        """
        return cls(fn, **kwargs)