# This code has been take from the professorship mechatronics at Saxion University
# It has been changed so it would work in this project
import numpy as np
# Taken from https://github.com/max-sn/robotics_foundation/tree/develop, see
# https://robotics-foundation.readthedocs.io/en/latest/python/num.html for docs


def vec_to_SO3(omega: np.array, theta: float = None) -> np.array:
    """
    'Integration' of an angular velocity to determine the orientation
    (expressed as rotation matrix) that this angular velocity reaches in unit
    time.

    Uses Rodrigues' formula to solve the matrix exponential of the vector's
    skew-symmetric matrix form.

    .. math::

        \\RotationMatrix =
        \\exp(\\TildeSkew{\\UnitLength{\\omega}}\\theta) =
        I + \\TildeSkew{\\UnitLength{\\omega}}\\sin\\theta +
        \\TildeSkew{\\UnitLength{\\omega}}^2(1-\\cos\\theta) \\in \\SOthree

    Args:
        omega: 3 vector angular velocity. Can be unit length, in that case
               theta should also be provided.
        theta: If omitted, the Euclidean norm of omega is assumed to be theta.

    Returns:
        3 by 3 rotation matrix :math:`\\RotationMatrix \\in \\SOthree`.

    See Also:
        :py:func:`vec_to_so3`
        :py:func:`SO3_to_vec`
    """
    if theta is None:
        # If no theta, take length of omega and make omega unit length.
        theta = np.linalg.norm(omega)
        omega = omega / theta

    # Skew-symmetric form of omega
    omega_tilde = vec_to_so3(omega)

    # Rodrigues' formula
    return np.eye(3) + np.sin(theta) * omega_tilde + \
        (1 - np.cos(theta)) * np.linalg.matrix_power(omega_tilde, 2)


def vec_to_so3(v: np.array) -> np.array:
    """
    Build a skew-symmetric matrix from a 3 vector.

    Args:
        v: 3 vector.

    Returns:
        3 by 3 skew-symmetric matrix form of v, :math:`\\TildeSkew{v}`.

    See Also:
        :py:func:`so3_to_vec`
    """
    return np.array([[0, -v.item(2), v.item(1)],
                     [v.item(2), 0, -v.item(0)],
                     [-v.item(1), v.item(0), 0]])


def inv_SE3(T: np.array) -> np.array:
    """
    Inverts the transformation matrix T.

    .. math::

        \\HomogeneousTransformationMatrix =
        \\begin{bmatrix}
        \\RotationMatrix & p \\\\
        0 & 1
        \\end{bmatrix}\\in\\SEthree
        \\quad\\Rightarrow\\quad
        \\HomogeneousTransformationMatrix^{-1} =
        \\begin{bmatrix}
        \\RotationMatrix\\Transposed & -\\RotationMatrix\\Transposed p \\\\
        0 & 1
        \\end{bmatrix}\\in\\SEthree

    Args:
        T: 4 by 4 homogeneous transformation matrix
           :math:`\\HomogeneousTransformationMatrix \\in \\SEthree`

    Returns:
        4 by 4 inverse of the homogeneous transformation matrix
        :math:`\\HomogeneousTransformationMatrix^{-1} \\in \\SEthree`
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3:4]

    return R_p_to_SE3(R.T, np.dot(-R.T, p))


def R_p_to_SE3(R: np.array, p: np.array) -> np.array:
    """
    Constructs a homogeneous transformation matrix from a rotation matrix and
    displacement vector.

    Args:
        R: 3 by 3 rotation matrix :math:`\\RotationMatrix\\in\\SOthree`
        p: 3 by 1 displacement vector

    Returns:
        4 by 4 homogeneous transformation matrix
        :math:`\\HomogeneousTransformationMatrix \\in \\SEthree`
    """
    return np.vstack((np.hstack((R, p)),
                      np.array([0, 0, 0, 1])))
