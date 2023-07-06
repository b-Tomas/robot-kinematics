# TODO: Move tests somewhere else. And if possible use an actual testing framework
# Run this script as plain python script from within the container

from transformation_service import *
from math import atan, atan2, cos, acos, sin, sqrt, floor, pi
import numpy as np


def test_transform():
    def test(transformed, expected):
        print(transformed, expected)
        for i in range(len(expected)):
            print(transformed[i], expected[i])
            assert floor(transformed[i]*100) == floor(expected[i]*100)

    transformed = transform((1, 0, 1, -pi/2), .5, 1, 1, .5)
    expected = (0, 0, 0, pi/2)
    assert len(transformed), len(expected)
    test(transformed, expected)
    
    transformed = transform((1.5, 0, 1.5, 0), .5, 1, 1, .5)
    expected = (0, 0, 0, 0)
    assert len(transformed), len(expected)
    test(transformed, expected)

    transformed = transform((0.274, 0, 0.2028, 0), 0.0595, 0.13023, 0.124, 0.126)
    expected = (0, 0, 0, 0)
    assert len(transformed), len(expected)
    test(transformed, expected)

    transformed = transform((0.286, 0, 0.2045, 0), 0.0595, 0.13023, 0.124, 0.126, 0.012, 0, 0.017)
    expected = (0, 0, 0, 0)
    assert len(transformed), len(expected)
    test(transformed, expected)


test_transform()

def general_mat(d, t, r, a):
    return np.mat([
        [cos(t), -sin(t)*cos(a), sin(t)*sin(a), r*cos(t)],
        [sin(t), cos(t)*cos(a), -cos(t)*sin(a), r*sin(t)],
        [0, sin(a), cos(a), d],
        [0, 0, 0, 1]
    ])
q1 = 0
q2 = 0
q3 = 0
q4 = 0
l1 = 0.0595
l2 = 0.13023
l3 = .124
l4 = .126
print(general_mat(l1, q1, 0, pi/2)*general_mat(0, q2+pi/2, l2, 0)*general_mat(0, q3-pi/2, l3, 0)*general_mat(0, q4, l4, 0))