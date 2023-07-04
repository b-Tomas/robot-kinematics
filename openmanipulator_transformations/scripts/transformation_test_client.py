#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from openmanipulator_transformations.srv import Transform, TransformRequest, TransformResponse


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
    if len(sys.argv) == 4:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        z = int(sys.argv[3])
    else:
        print(f"Usage: {sys.argv[0]} x y z")
        sys.exit(1)
    coords = (x, y, z)
    print(f"Requesting: {coords}")
    print(f"Response: {transform_client(coords)}")
