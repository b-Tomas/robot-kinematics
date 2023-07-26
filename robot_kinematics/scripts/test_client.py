#!/usr/bin/env python3

import sys
import rospy
from robot_kinematics.srv import Transform

# Use as a python script
# Call the transformations server and print the result

TRANSFORM_SERVICE_NAME = "/kinematics/inverse_transformation"


def transform_client(vec):
    rospy.wait_for_service(TRANSFORM_SERVICE_NAME)
    try:
        service = rospy.ServiceProxy(TRANSFORM_SERVICE_NAME, Transform)
        resp = service(vec)
        return resp.joint_angles
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


if __name__ == "__main__":
    if len(sys.argv) == 5:
        coords = tuple(map(lambda x: float(x), sys.argv[1:]))
    else:
        print(f"Usage: {sys.argv[0]} x y z t")
        sys.exit(1)
    print(f"Requesting: {coords}")
    print(f"Response: {transform_client(coords)}")
