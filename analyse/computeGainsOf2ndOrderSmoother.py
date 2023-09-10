
#
# File: computeTrajectroyGains.cpp
# Author: Burak Yueksel
# Date: 2023-09-10
#
# Python re-implementation of this: https://github.com/burakyueksel/controls/blob/main/Quadrotor_Control_All/simulate/place_2nd_by.m
# implemented based on the Ackermann's formula.
# see e.g. the page 6 of
# https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-30-feedback-control-systems-fall-2010/lecture-notes/MIT16_30F10_lec11.pdf

import numpy as np

def place_2nd_by(A, B, poles):
    if len(poles) != 2:
        print("Poles should be of size 2")
        return None

    if A.shape != (2, 2):
        print("A Matrix should be of size 2x2")
        return None

    if B.shape != (2, 1):
        print("B Matrix should be of size 2x1")
        return None

    myinf = 10000
    negpoles = -np.array(poles)

    coef_2 = 1
    coef_1 = negpoles[0]+negpoles[1]
    coef_0 = negpoles[0]*negpoles[1]

    Mc = np.hstack([B, A @ B])

    absDetMc = np.abs(np.linalg.det(Mc))

    print("Det of Mc:")
    print(absDetMc)

    if absDetMc > 1e-6:
        K = np.dot(np.array([0, 1]), np.linalg.inv(Mc)) @ (
                coef_2 * np.linalg.matrix_power(A, 2) +
                coef_1 * A +
                coef_0 * np.eye(2))
    else:
        print("System is not controllable")
        K = myinf * np.eye(2)

    return K

# Example usage with poles
poles = [-1.0, -2.]
A = np.array([[0, 1],
              [0, 0]])

B = np.array([[0],
              [1]])

K = place_2nd_by(A, B, poles)
print("Optimal Gain Matrix K:")
print(K)
print(A)
