
#
# File: computeTrajectroyGains.cpp
# Author: Burak Yueksel
# Date: 2023-09-10
#
# Python re-implementation of this: https://github.com/burakyueksel/controls/blob/main/Quadrotor_Control_All/simulate/place_4th_by.m
# implemented based on the Ackermann's formula.
# see e.g. the page 6 of
# https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-30-feedback-control-systems-fall-2010/lecture-notes/MIT16_30F10_lec11.pdf

import numpy as np

def place_4th_by(A, B, poles):
    if len(poles) != 4:
        print("Poles should be of size 4")
        return None

    if A.shape != (4, 4):
        print("A Matrix should be of size 4x4")
        return None

    if B.shape != (4, 1):
        print("B Matrix should be of size 4x1")
        return None

    myinf = 10000
    negpoles = -np.array(poles)

    coef_4 = 1
    coef_3 = negpoles[0]+negpoles[1]+negpoles[2]+negpoles[3]
    coef_2 = negpoles[0]*negpoles[1]+negpoles[2]*negpoles[3]+(negpoles[0]+negpoles[1])*(negpoles[2]+negpoles[3])
    coef_1 = (negpoles[0]+negpoles[1])*negpoles[2]*negpoles[3] + negpoles[0]*negpoles[1]*(negpoles[2]+negpoles[3])
    coef_0 = negpoles[0]*negpoles[1]*negpoles[2]*negpoles[3]

    Mc = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B])

    absDetMc = np.abs(np.linalg.det(Mc))

    print("Det of Mc:")
    print(absDetMc)

    if absDetMc > 1e-6:
        K = np.dot(np.array([0, 0, 0, 1]), np.linalg.inv(Mc)) @ (
                coef_4 * np.linalg.matrix_power(A, 4) +
                coef_3 * np.linalg.matrix_power(A, 3) +
                coef_2 * np.linalg.matrix_power(A, 2) +
                coef_1 * A +
                coef_0 * np.eye(4))
    else:
        print("System is not controllable")
        K = myinf * np.eye(4)

    return K

# Example usage with poles
poles = [-1.0, -2, -3.0, -4.0]
A = np.array([[0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

B = np.array([[0],
              [0],
              [0],
              [1]])

K = place_4th_by(A, B, poles)
print("Optimal Gain Matrix K:")
print(K)
print(A)
