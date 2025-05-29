import numpy as np
from geometry_msgs.msg import Point

def wedge_SO3(xi: np.ndarray):
    xi = np.array(xi).ravel()
    X = np.array(
        [
            [0, -xi[2], xi[1]],
            [xi[2], 0, -xi[0]],
            [-xi[1], xi[0], 0],
        ]
    )
    return X

def Exp_SO3(xi: np.ndarray):
    """
    Maps elements of the vector Lie algebra so(3) to the group.
    """
    phi = np.array(xi).ravel()
    angle = np.linalg.norm(phi)

    # Use Taylor series expansion
    if angle < 1e-6:
        t2 = angle**2
        A = 1.0 - t2 / 6.0 * (1.0 - t2 / 20.0 * (1.0 - t2 / 42.0))
        B = (
            1.0
            / 2.0
            * (1.0 - t2 / 12.0 * (1.0 - t2 / 30.0 * (1.0 - t2 / 56.0)))
        )
    else:
        A = np.sin(angle) / angle
        B = (1.0 - np.cos(angle)) / (angle**2)

    # Rodirgues rotation formula (103)
    Xi = wedge_SO3(phi)
    return np.eye(3) + A * Xi + B * Xi.dot(Xi)

def to_quat(C, order="wxyz"):
    """
    Returns the quaternion corresponding to DCM C.
    Parameters
    ----------
    C : ndarray with shape (3,3)
        DCM/rotation matrix to convert.
    order : str, optional
        quaternion element order "xyzw" or "wxyz", by default "wxyz"
    Returns
    -------
    ndarray with shape (4,1)
            quaternion representation of C
    Raises
    ------
    ValueError
        if `C` does not have shape (3,3)
    ValueError
        if `order` is not "xyzw" or "wxyz"
    """

    C = C.reshape((3, 3))
    if C.shape != (3, 3):
        raise ValueError("C must have shape (3,3).")

    eta = 0.5 * (np.trace(C) + 1) ** 0.5
    eps = -np.array(
        [C[1, 2] - C[2, 1], C[2, 0] - C[0, 2], C[0, 1] - C[1, 0]]
    ) / (4 * eta)

    if order == "wxyz":
        q = np.hstack((eta, eps)).reshape((-1, 1))
    elif order == "xyzw":
        q = np.hstack((eps, eta)).reshape((-1, 1))
    else:
        raise ValueError("order must be 'wxyz' or 'xyzw'. ")

    return q

def dcm_to_quaternion(C: np.ndarray):
    """Converts a DCM to a quaternion, where the last component is the scalar."""
    eta = 0.5 * (np.trace(C) + 1) ** 0.5

    epsilon = np.zeros((3, 1))
    if abs(eta) < 1e-7:
        eta = 1e-16

    epsilon[0, 0] = -(C[1, 2] - C[2, 1]) / (4 * eta)
    epsilon[1, 0] = -(C[2, 0] - C[0, 2]) / (4 * eta)
    epsilon[2, 0] = -(C[0, 1] - C[1, 0]) / (4 * eta)

    quaternion = np.vstack((epsilon, eta))
    return quaternion


def numpy_to_point(array: np.ndarray):
    """Converts a 3x1 numpy array to a Point msg."""
    x, y, z = array.flatten()

    return Point(x, y, z)
