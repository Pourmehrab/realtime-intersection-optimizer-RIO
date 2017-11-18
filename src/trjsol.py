'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Last update: November/2017
'''

import numpy as np


def LVTOsol(d0: float, v0: float, vmax: float, amin: float, amax: float, gs: float, gt: float, simObj):
    # these variables will store the optimal solution at the end of this function
    t1star, t2star, t3star, v2star, a1star, v3star, a3star, tstar = 0

    # flag becomes true once we have the optimal solution
    flag = False

    # EDGES CAN BE MADE WITH MOVING ALONG v2
    v3 = vmax
    for a1 in (amin, amax):
        for a3 in (amin, amax):
            if abs(a1 - a3) > 0.01:  # when a1 and a3 are not equal
                np.power(2, 3)
                undrsqrt = (a1 * (np.power(v3, 2) - 2 * a3 * d0) - a3 * np.power(v0, 2)) / (a1 - a3)
                if undrsqrt > 0:
                    v2 = np.sqrt(undrsqrt)
                    if v2 <= vmax:
                        v2star, v3star, a1star, a3star, tstar, flag = testsol(v2, v3, a1, a3, d0, v0, v2star, v3star,
                                                                              a1star, a3star, tstar, flag)
            for t in (gs, gt):
                v2 = v1edge(d0, v0, v3, a1, a3, vmax, t)
                v2star, v3star, a1star, a3star, tstar, flag = testsol(v2, v3, a1, a3, d0, v0, v2star, v3star, a1star,
                                                                      a3star, tstar, flag)
    if flag:
        t1star = np.round((v2star - v0) / a1, 2)
        t2star = np.round(
            (d0 - (np.power(v2, 2) - np.power(v0, 2)) / (2 * a1) - (np.power(v3, 2) - np.power(v2, 2)) / (2 * a3)) / v2,
            2)
        t3star = np.round((v3star - v2) / a3, 2)

        return t1star, t2star, t3star, a1star, v2star, a3star, v3star, tstar
    else:
        print('LVTO solver has failed.')
        quit()


def testsol(v2, v3, a1, a3, d0, v0, v2star, v3star, a1star, a3star, tstar, flag):
    if (v2 - v0) * a1 > 0 or (v3 - v2) * a3 > 0:
        # Solution is not feasible
        return v2star, v3star, a1star, a3star, tstar, flag

    T = d0 / v2 - (v3 - v2) ^ 2 / (2 * a3 * v2) + (v2 - v0) ^ 2 / (2 * a1 * v2)
    if T < tstar:
        return v2, v3, a1, a3, T, True
    else:
        return v2star, v3star, a1star, a3star, tstar, flag


def v1edge(d0, v0, v3, a1, a3, vmax, t):
    upbound = vmax
    undrsqrt = a1 * a3 * (
        a1 * (np.power(t, 2) * a3 + 2 * d0 - 2 * t * v3) + np.power(v3 - v0, 2) - 2 * a3 * (d0 - t * v0))
    if undrsqrt >= 0:
        b = a1 * (t * a3 - v3) + a3 * v0

        upbound1, upbound2 = (b + np.sqrt(undrsqrt)) / (a3 - a1), (b - np.sqrt(undrsqrt)) / (a3 - a1)
        upperBound = compbounds(upbound1, upbound2, vmax)


def compbounds(q1, q2, q):
    prodsign = np.sign(q1) * np.sign(q2)
    if prodsign > 0 and q1 > 0:
        return min(q, q1, q2)
    elif prodsign < 0:
        return min(q, max(q1, q2))
    else:
        return q
