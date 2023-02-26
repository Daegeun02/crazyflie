## generate acc command to tracking trajectory
from numpy import array, zeros

from time import sleep



## read constants
from constants import read_constant

_const = read_constant('gain')
Kp     = _const["Kp"]
Kd     = _const["Kd"]

_const = read_constant('gravity')
g      = _const["g"]

print(Kp)
print(Kd)


## extern variable
posvel = zeros(6)
acccmd = zeros(3)
P_pos  = zeros(3)
D_pos  = zeros(3)
careg  = array([0,0,g])


def generate_acccmd(cf, des, dt=0.1):

    posvel[:] = cf.posvel

    cf.destination[:] = des

    P_pos[:] = des[:3] - posvel[:3]
    D_pos[:] = des[3:] - posvel[3:]

    acccmd[:] = 0
    acccmd[:] += P_pos * Kp
    acccmd[:] += D_pos * Kd
    acccmd[:] += careg

    return acccmd