## This file contains functions that transform coordinate
from numpy import cos, sin, sqrt
from numpy import arctan2, arcsin



def euler2quatn(euler):
    ## numpy function only get [rad] as input of function
    ## r: roll, p: pitch, y: yaw
    r, p, y = euler

    ## basic
    r, p, y = r/2, p/2, y/2         ## conversion
    cr, sr = cos(r), sin(r)
    cp, sp = cos(p), sin(p)
    cy, sy = cos(y), sin(y)

    ## quaternions
    q0 = cy*cp*cr + sy*sp*sr
    q1 = cy*cp*sr - sy*sp*cr
    q2 = cy*sp*cr - sy*cp*sr
    q3 = sy*cp*cr - cy*sp*sr

    ## normalize
    Q = sqrt(q0**2 + q1**2 + q2**2 + q3**3)
    q0 /= Q
    q1 /= Q
    q2 /= Q
    q3 /= Q

    ## return
    return q0, q1, q2, q3


def quatn2euler(quatn):
    ## it's complicate
    q0, q1, q2, q3 = quatn

    ## 1. roll
    ry = q0 * q1 + q2 * q3
    rx = q0**2 + q3**2 - 0.5

    roll = arctan2( ry, rx )

    ## 2. pitch
    pa = 2 * (q0 * q2 - q1 * q3)

    pitch = arcsin( pa )

    ## 3. yaw
    yy = q0 * q3 + q1 * q2
    yx = q0**2 + q1**2 - 0.5

    yaw = arctan2( yy, yx )

    ## return 
    return roll, pitch, yaw

def NED2euler(NED):
    ## zz
    N, E, D = NED