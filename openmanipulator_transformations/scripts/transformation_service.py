#!/usr/bin/env python3

import rospy

from openmanipulator_transformations.srv import Transform, TransformRequest, TransformResponse
import numpy as np
from math import atan, atan2, cos, acos, sin, sqrt, pi

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
LINK_2_LONGITUDE = np.linalg.norm(JOINT_2_ORIGIN)
LINK_3_LONGITUDE = np.linalg.norm(JOINT_3_ORIGIN)
LINK_4_LONGITUDE = np.linalg.norm(JOINT_4_ORIGIN)
# Distance from joint 4 to the center of the end-effector (the red cube in gazebo)
LAST_BIT_LONGITUDE =  np.linalg.norm(JOINT_END_EFFECTOR)

# These are the final parameters used for computation
L1 = LINK_2_LONGITUDE
L2 = LINK_3_LONGITUDE
L3 = LINK_4_LONGITUDE
L4 = LAST_BIT_LONGITUDE
OFFSET_X, OFFSET_Y, OFFSET_Z = JOINT_1_ORIGIN


def transform(vec: tuple, l1, l2, l3, l4, offset_x=0, offset_y=0, offset_z=0) -> tuple:
    x, y, z, t = vec

    x = x - offset_x
    y = y - offset_y
    z = z - l1 - offset_z # Height of the second joint. This math is relative to the second joint

    # Transform (X, Y, Z, t) -> (X', Z') the coordinates in the plane formed by the joints with the origin in the first joint
    # X' being the horizontal distance from the origin to the end position
    # We are using cylindrical coordinates now
    x_t = sqrt(x**2 + y**2)
    z_t = z

    # Solve the equation system formed by the combination of the forward kinematics transformations
    w1 = x_t-l4*cos(t)
    w2 = z_t-l4*sin(t)

    a1 = 2*w1*l2
    a2 = 2*w2*l2
    a3 = w1**2+w2**2+l2**2-l3**2

    t1 = atan2(a2, a1) + acos(a3/sqrt(a1**2+a2**2))
    t2 = atan2((w2-l2*sin(t1)), (w1-l2*cos(t1))) - t1
    t3 = t-t1-t2

    # Transform back to cartesian coordinates and apply initial conditions
    q1 = atan2(y, x)
    aux = atan(JOINT_3_ORIGIN[0]/JOINT_3_ORIGIN[2])
    q2 =  pi/2-aux-t1
    q3 = -pi/2-t2 + aux
    q4 = -t3

    return (q1, q2, q3, q4)


def process_service_request(req: TransformRequest) -> TransformResponse:
    rospy.loginfo(f"Received request: {req}")
    if len(req.coordinates) != 4: raise ValueError  # TODO: use a more meaningful error
    response = TransformResponse()
    response.joint_angles = transform(req.coordinates, L1, L2, L3, L4, OFFSET_X, OFFSET_Y, OFFSET_Z)
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
