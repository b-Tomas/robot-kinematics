#!/usr/bin/env python3

import rospy

from openmanipulator_transformations.srv import Transform, TransformRequest, TransformResponse
import numpy as np

TRANSFORM_SERVICE_NAME = "/transformations/transform"
TRANSFORM_NODE_NAME = "transform_srv"


# Computation service. Converts cartesian coordinates to joint angles using the matrix below

matrix = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])


def transform(vec: tuple) -> tuple:
    # Operation:
    #   matrix (4 by 3) . vec (3 by 1)
    # Vec should be a one dimensional array of three elements
    return np.matmul(matrix, np.asarray(vec))


def process_service_request(req: TransformRequest) -> TransformResponse:
    rospy.loginfo(f"Received request: {req}")
    response = TransformResponse()
    response.joint_angles = transform(req.coordinates)
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
