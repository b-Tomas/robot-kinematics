#!/usr/bin/env python3

from kinematics import utils
from numpy import pi, round
import unittest
import rostest


def round_collection(arr: tuple, decimals=3) -> tuple:
    # round the result to the (10^decimals)th
    return tuple(round(a * (10**decimals)) / (10**decimals) for a in arr)


class Tests(unittest.TestCase):
    # Test transformation math

    def test_forward_transformation(self):
        # Simple robot with no joint offset
        seg_longitudes = (0.5, 1, 1, 0.5)
        joint_angles = (0, 0, 0, 0)
        joint_offsets = (0, 0, 0, 0)
        result = round_collection(
            utils.forward_transform(seg_longitudes, joint_angles, joint_offsets)
        )
        expected = (2.5, 0, 0.5)
        assert result == expected

        # Simple robot with joint offset
        seg_longitudes = (0.5, 1, 1, 0.5)
        joint_angles = (0, 0, 0, 0)
        joint_offsets = (0, -pi / 2, pi / 2, 0)
        result = round_collection(
            utils.forward_transform(seg_longitudes, joint_angles, joint_offsets)
        )
        expected = (1.5, 0, 1.5)
        assert result == expected

        # Simple robot with joint offset in not-home pose
        seg_longitudes = (0.5, 1, 1, 0.5)
        joint_angles = (0, 0, 0, pi / 2)
        joint_offsets = (0, -pi / 2, pi / 2, 0)
        result = round_collection(
            utils.forward_transform(seg_longitudes, joint_angles, joint_offsets)
        )
        expected = (1, 0, 1)
        assert result == expected

        # The real robot
        # Values taken from the calculations in src/kinematics/config.py
        seg_longitudes = (0.06, 0.13, 0.124, 0.126)
        joint_angles = (0, 0, 0, 0)
        joint_offsets = (0, -1.385, 1.385, 0)
        result = round_collection(
            utils.forward_transform(seg_longitudes, joint_angles, joint_offsets)
        )
        # expected = home pose - origin offset = (0.286-0.012, 0, 0.205-0.017)
        expected = (0.274, 0, 0.188)
        assert result == expected


if __name__ == "__main__":
    rostest.run("openmanipulator_transformations", "kinematics math tests", Tests)
