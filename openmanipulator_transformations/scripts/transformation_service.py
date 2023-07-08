#!/usr/bin/env python3

import rospy

from openmanipulator_transformations.srv import (
    Transform,
    TransformRequest,
    TransformResponse,
)
from numpy import arctan2, array, cos, arccos, sin
from numpy.linalg import norm

# Computation service. Converts cartesian coordinates to joint angles using the data below

TRANSFORM_SERVICE_NAME = "/transformations/transform"
TRANSFORM_NODE_NAME = "transform_srv"

# Distances between joints
# Obtained from https://github.com/ROBOTIS-GIT/open_manipulator/blob/07625015a0088a069575f7e6e74eb931edce9ed3/open_manipulator_description/urdf/open_manipulator.urdf.xacro
# The origins of the joints are relative to the origin of the parent link

# Child of link1, parent of link2. Rotates in the XY plane
JOINT_1_ORIGIN = array([0.012, 0, 0.017])
# Child of link2, parent of link3. Rotates in the XZ plane
JOINT_2_ORIGIN = array([0, 0, 0.0595])
# Child of link3, parent of link4. Rotates in the XZ plane
JOINT_3_ORIGIN = array([0.024, 0, 0.128])
# Child of link3, parent of link5 (between joint 3 and gripper joint). Rotates in the XZ plane
JOINT_4_ORIGIN = array([0.124, 0, 0])
# Child of link5, which starts at joint4
JOINT_END_EFFECTOR = array([0.126, 0, 0])

# Link2 is the one above joint1 and because of that it rotates around its longitudinal axis. It is always pointing up
LINK_2_LONGITUDE = norm(JOINT_2_ORIGIN)
LINK_3_LONGITUDE = norm(JOINT_3_ORIGIN)
LINK_4_LONGITUDE = norm(JOINT_4_ORIGIN)
# Distance from joint 4 to the center of the end-effector (the red cube in gazebo)
LAST_BIT_LONGITUDE = norm(JOINT_END_EFFECTOR)

# These are the final parameters used for computation
LINK_LONGITUDES = (
    LINK_2_LONGITUDE,
    LINK_3_LONGITUDE,
    LINK_4_LONGITUDE,
    LAST_BIT_LONGITUDE,
)
OFFSET_XYZ = tuple(JOINT_1_ORIGIN)

# If you look closely, you will notice that joints 2 and 3 angles are not exactly at a 90° relative to fully extended
# This is how we calculate the angle
offset = arctan2(JOINT_3_ORIGIN[2], JOINT_3_ORIGIN[0])
JOINT_OFFSETS = (0, -offset, offset, 0)


def transform(
    ee_pose: tuple,
    seg_longitudes: tuple,
    offset_joints: tuple,
    offset_xyz=(0, 0, 0),
) -> tuple:
    """Takes an end-effector position in space and computes the robot parameters for it to reach that position by
    using inverse kinematics calculations.
    If the position is out of reach, it raises ValueError

    Args:
        ee_pose (tuple): (x, y, z) position of the end-effector
        seg_longitudes (tuple): Longitude of each segment of the robot
        offset_joints (tuple): Zero angle of each joint relative to a fully extended horizontal position
        offset_xyz (tuple, optional): Position of the first joint in the frame of the end-effector. Defaults to (0, 0, 0).

    Returns:
        tuple: Required angle for each joint
    """
    cant_joints = 4
    assert len(ee_pose) == 4
    assert len(seg_longitudes) == cant_joints
    assert len(offset_joints) == cant_joints
    assert len(offset_xyz) == 3

    x, y, z, t = ee_pose
    l1 = seg_longitudes[0]
    offset_x, offset_y, offset_z = offset_xyz

    # Apply robot position offsets
    x = x - offset_x
    y = y - offset_y
    # Height of the second joint. This math is relative to the second joint
    z = z - l1 - offset_z

    # Obtain the parameters for a fully extended robot
    t1, t2, t3, t4 = transformation((x, y, z, t), seg_longitudes)

    # Transform back to cartesian coordinates and apply initial conditions
    offset_q1, offset_q2, offset_q3, offset_q4 = offset_joints
    q1 = t1 - offset_q1
    q2 = -t2 - offset_q2
    q3 = -t3 - offset_q3
    q4 = -t4 - offset_q4

    return (q1, q2, q3, q4)


def transformation(
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


def process_service_request(req: TransformRequest) -> TransformResponse:
    rospy.loginfo(f"Received request: {req}")
    assert len(req.coordinates) == 4, "Array length must be four"
    response = TransformResponse()
    response.joint_angles = transform(
        ee_pose=req.coordinates,
        seg_longitudes=LINK_LONGITUDES,
        offset_joints=JOINT_OFFSETS,
        offset_xyz=OFFSET_XYZ,
    )
    rospy.loginfo(f"Generated response: {response}")
    return response


def serve():
    # ROS node for the service server.
    rospy.init_node(TRANSFORM_NODE_NAME, anonymous=False)

    # Create a ROS service type.
    rospy.Service(TRANSFORM_SERVICE_NAME, Transform, process_service_request)

    # Log message about service availability.
    rospy.loginfo(f"{TRANSFORM_SERVICE_NAME} service is now available.")
    rospy.spin()


if __name__ == "__main__":
    serve()
