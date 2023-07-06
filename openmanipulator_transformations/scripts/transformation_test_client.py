#!/usr/bin/env python3

import sys
import rospy
from openmanipulator_transformations.srv import Transform


TRANSFORM_SERVICE_NAME = "/transformations/transform"

# This is a proof of concept script. It is not meant to be used for anything in particular
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
