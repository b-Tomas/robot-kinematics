#!/usr/bin/env python3

import rospy
from kinematics.config import (
    INV_TRANSFORM_NODE_NAME,
    INV_TRANSFORM_SERVICE_NAME,
    JOINT_OFFSETS,
    LINK_LONGITUDES,
    OFFSET_XYZ,
)
from kinematics.utils import inverse_transform

from robot_kinematics.srv import Transform, TransformRequest, TransformResponse

"""
Inverse kinematics computation service. Converts cartesian coordinates to joint angles using the parameters in config.py
"""


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
    t1, t2, t3, t4 = inverse_transform((x, y, z, t), seg_longitudes)

    # Transform back to cartesian coordinates and apply initial conditions
    offset_q1, offset_q2, offset_q3, offset_q4 = offset_joints
    q1 = t1 - offset_q1
    q2 = -t2 - offset_q2
    q3 = -t3 - offset_q3
    q4 = -t4 - offset_q4

    return (q1, q2, q3, q4)


def process_service_request(req: TransformRequest) -> TransformResponse:
    rospy.loginfo(f"Received request: {req}")
    assert len(req.input) == 4, "Array length must be four"
    response = TransformResponse()
    response.output = transform(
        ee_pose=req.input,
        seg_longitudes=LINK_LONGITUDES,
        offset_joints=JOINT_OFFSETS,
        offset_xyz=OFFSET_XYZ,
    )
    rospy.loginfo(f"Generated response: {response}")
    return response


def serve():
    # ROS node for the service server.
    rospy.init_node(INV_TRANSFORM_NODE_NAME, anonymous=False)

    # Create a ROS service type.
    rospy.Service(INV_TRANSFORM_SERVICE_NAME, Transform, process_service_request)

    # Log message about service availability.
    rospy.loginfo(f"{INV_TRANSFORM_SERVICE_NAME} service is now available.")
    rospy.spin()


if __name__ == "__main__":
    serve()
