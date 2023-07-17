from numpy import arctan2, cos, arccos, sin, pi
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


def forward_transformation():
    # TODO
    pass


def unwrap_angles(vec: list) -> list:
    # Adjust positions within the range of -pi to pi for the robot's movement

    for i in range(len(vec)):
        while vec[i] > pi or vec[i] < -pi:
            if vec[i] > pi:
                vec[i] -= 2 * pi
            elif vec[i] < -pi:
                vec[i] += 2 * pi

    return vec
