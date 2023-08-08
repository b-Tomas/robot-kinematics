#!/usr/bin/env python3

import rospy
from kinematics.config import (
    FWD_TRANSFORM_NODE_NAME,
    FWD_TRANSFORM_SERVICE_NAME,
    JOINT_OFFSETS,
    LINK_LONGITUDES,
    OFFSET_XYZ,
)
from kinematics.utils import forward_transform
from numpy import array

from robot_kinematics.srv import Transform, TransformRequest, TransformResponse

"""
Forward kinematics computation service. Converts joint angles to cartesian coordinates using the parameters in config.py
"""


def process_service_request(req: TransformRequest) -> TransformResponse:
    rospy.loginfo(f"Received request: {req}")
    assert len(req.input) == 4, "Array length must be four"
    response = TransformResponse()
    response.output = (
        array(forward_transform(LINK_LONGITUDES, req.input, JOINT_OFFSETS)) + OFFSET_XYZ
    )
    rospy.loginfo(f"Generated response: {response}")
    return response


def serve():
    # ROS node for the service server.
    rospy.init_node(FWD_TRANSFORM_NODE_NAME, anonymous=False)

    # Create a ROS service type.
    rospy.Service(FWD_TRANSFORM_SERVICE_NAME, Transform, process_service_request)

    # Log message about service availability.
    rospy.loginfo(f"{FWD_TRANSFORM_SERVICE_NAME} service is now available.")
    rospy.spin()


if __name__ == "__main__":
    serve()
