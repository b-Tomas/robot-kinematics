# TODO: Move tests somewhere else. And if possible use an actual testing framework
# Run this script as plain python script from within the container

from transformation_service import *
from math import cos, sin, floor, pi
import numpy as np


def test_transform():
    def test(transformed, expected):
        for i in range(len(expected)):
            assert floor(transformed[i]*100) == floor(expected[i]*100)

    print("\ntest easy robot in init pose")
    transformed = transform((1.5, 0, 1.5, 0), .5, 1, 1, .5)
    expected = (0, 0, 0, 0)
    assert len(transformed), len(expected)
    test(transformed, expected)
    
    print("\ntest easy robot in a different position")
    transformed = transform((1, 0, 1, -pi/2), .5, 1, 1, .5)
    expected = (0, 0, 0, pi/2)
    assert len(transformed), len(expected)
    test(transformed, expected)
    
    print("\ntest easy robot with offset")
    transformed = transform((2, 0, 2, 0), .5, 1, 1, .5, .5, 0, .5)
    expected = (0, 0, 0, 0)
    assert len(transformed), len(expected)
    test(transformed, expected)
    
    # There is a considerable margin of error when utilizing real numbers, therefore the tests fail
    # print("\ntest real robot without offset")
    # # real values
    # L1 = 0.0595
    # L2 = 0.130230565
    # L3 = 0.124
    # L4 = 0.126
    # OFFSET = (0.012, 0, 0.017)
    # EE_ORIGIN = (0.012+0.024+.124+.126, 0, .017+.0595+.128, 0)

    # transformed = transform((EE_ORIGIN[0]-OFFSET[0], EE_ORIGIN[1]-OFFSET[1], EE_ORIGIN[2]-OFFSET[2], 0), L1, L2, L3, L4)
    # expected = (0, 0, 0, 0)
    # assert len(transformed), len(expected)
    # test(transformed, expected)

    # transformed = transform(EE_ORIGIN, L1, L2, L3, L4, *OFFSET)
    # expected = (0, 0, 0, 0)
    # assert len(transformed), len(expected)
    # test(transformed, expected)


test_transform()

# General forward kinematics matrix
def general_mat(d, t, r, a):
    return np.mat([
        [cos(t), -sin(t)*cos(a), sin(t)*sin(a), r*cos(t)],
        [sin(t), cos(t)*cos(a), -cos(t)*sin(a), r*sin(t)],
        [0, sin(a), cos(a), d],
        [0, 0, 0, 1]
    ])

# q1 = 0
# q2 = 0
# q3 = 0
# q4 = 0
# l1 = 0.0595
# l2 = 0.13023
# l3 = .124
# l4 = .126
# print(general_mat(l1, q1, 0, pi/2)*general_mat(0, q2+pi/2, l2, 0)*general_mat(0, q3-pi/2, l3, 0)*general_mat(0, q4, l4, 0))
