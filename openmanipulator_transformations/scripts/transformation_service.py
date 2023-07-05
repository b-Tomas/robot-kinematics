#!/usr/bin/env python3

import rospy

from openmanipulator_transformations.srv import Transform, TransformRequest, TransformResponse
import numpy as np

# Computation service. Converts cartesian coordinates to joint angles using the data below

TRANSFORM_SERVICE_NAME = "/transformations/transform"
TRANSFORM_NODE_NAME = "transform_srv"

# Distances between joints
# Obtained from https://github.com/ROBOTIS-GIT/open_manipulator/blob/07625015a0088a069575f7e6e74eb931edce9ed3/open_manipulator_description/urdf/open_manipulator.urdf.xacro
# The origins of the joints are relative to the origin of the parent link
JOINT_1_ORIGIN = np.array([0.012, 0, 0.017])  # Child of link1, parent of link2. Rotates in the XY plane
JOINT_2_ORIGIN = np.array([0,     0, 0.0595]) # Child of link2, parent of link3. Rotates in the XZ plane
JOINT_3_ORIGIN = np.array([0.024, 0, 0.128])  # Child of link3, parent of link4. Rotates in the XZ plane
JOINT_4_ORIGIN = np.array([0.124, 0, 0])      # Child of link3, parent of link5 (between joint 3 and gripper joint). Rotates in the XZ plane
JOINT_END_EFFECTOR = np.array([0.126, 0, 0])  # Child of link5, which starts at joint4

# Link2 is the one above joint1 and because of that it rotates around its longitudinal axis. It is always pointing up
LINK_2_LONGITUDE =  np.linalg.norm(JOINT_2_ORIGIN)
LINK_3_LONGITUDE =  np.linalg.norm(JOINT_3_ORIGIN)
LINK_4_LONGITUDE =  np.linalg.norm(JOINT_4_ORIGIN)
# Distance from joint 4 to the center of the end-effector (the red cube in gazebo)
LAST_BIT_LONGITUDE =  np.linalg.norm(JOINT_END_EFFECTOR)

matrix = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
    [0, 0, 0],
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
