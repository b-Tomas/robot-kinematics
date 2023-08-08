from numpy import arccos, arctan2, array, asarray, cos, mat, matrix, pi, sin, squeeze
from numpy.linalg import norm


def inverse_transform(
    ee_pose: tuple,
    seg_longitudes: tuple,
) -> tuple:
    """Trasforms robot parameters to cartesian coordinates for a fully extended robot centered at the origin

    Args:
        ee_pose (tuple): end-effector pose
        seg_longitudes (tuple): longitudes of eack link

    Returns:
        tuple: Robot parameters
    """
    x, y, z, t = ee_pose
    # The length of the first link is not useful because of the orientation of the first joint
    _, l2, l3, l4 = seg_longitudes

    # Transform (X, Y, Z, t) -> (X', Z') the coordinates in the plane formed by the joints with the origin in the first joint
    # X' being the horizontal distance from the origin to the end position
    # We are using cylindrical coordinates now
    x_t = norm([x, y])
    z_t = z

    # Solve the equation system formed by the combination of the forward kinematics transformations
    w1 = x_t - l4 * cos(t)
    w2 = z_t - l4 * sin(t)

    a1 = 2 * w1 * l2
    a2 = 2 * w2 * l2
    a3 = w1**2 + w2**2 + l2**2 - l3**2

    # Raises ValueError if the position is out of reach
    t1 = arctan2(a2, a1) + arccos(a3 / norm([a1, a2]))
    t2 = arctan2((w2 - l2 * sin(t1)), (w1 - l2 * cos(t1))) - t1
    t3 = t - t1 - t2

    # Transform back to cartesian coordinates and apply initial conditions
    q1 = arctan2(y, x)
    q2 = t1
    q3 = t2
    q4 = t3

    return (q1, q2, q3, q4)


def general_mat(t, d, r, a) -> matrix:
    """General forward kinematics matrix
    Obtained in section `2.2. Parámetros de Denavit-Hartenberg` of the documentation
    """
    return mat(
        [
            [cos(t), -sin(t) * cos(a), sin(t) * sin(a), r * cos(t)],
            [sin(t), cos(t) * cos(a), -cos(t) * sin(a), r * sin(t)],
            [0, sin(a), cos(a), d],
            [0, 0, 0, 1],
        ]
    )


def forward_transform(seg_longitudes: tuple, joint_angles: tuple, joint_offsets: tuple) -> tuple:
    """Retunrns the position of the end-effector based on the description of the robot
    The shape of the robot is the described in section `2.2. Parámetros de Denavit–Hartenberg`
    of the documentation

    Args:
        each tuple must have length 4
        seg_longitudes (tuple): distance between each pair of joints
        joint_angles (tuple): robot parameters
        joint_offsets (tuple): joint origin offsets relative to a horizontally extended robot

    Returns:
        tuple: (x, y, z) position
    """
    l1, l2, l3, l4 = seg_longitudes
    q1, q2, q3, q4 = joint_angles
    p1, p2, p3, p4 = joint_offsets
    m = (
        general_mat(q1 + p1, l1, 0, -pi / 2)
        * general_mat(q2 + p2, 0, l2, 0)
        * general_mat(q3 + p3, 0, l3, 0)
        * general_mat(q4 + p4, 0, l4, 0)
    )
    x, y, z, _ = squeeze(asarray(m @ array([0, 0, 0, 1])))
    return (x, y, z)


def unwrap_angles(vec: list) -> list:
    """Unwrap the elements of `vec` to the range -pi to pi

    Args:
        vec (list): list of angles

    Returns:
        list: unwrapped angles
    """

    for i in range(len(vec)):
        while vec[i] > pi or vec[i] < -pi:
            if vec[i] > pi:
                vec[i] -= 2 * pi
            elif vec[i] < -pi:
                vec[i] += 2 * pi

    return vec
