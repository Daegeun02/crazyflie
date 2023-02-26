## generate acc command to tracking trajectory
from numpy import array, zeros

from time import sleep



## read constants
from constants import read_constant

_const = read_constant('gain')
Kp     = _const["Kp"]
Kd     = _const["Kd"]

print(Kp)
print(Kd)


## extern variable
posvel = zeros(6)
acccmd = zeros(6)


def generate_acccmd(cf, des, T, dt=0.1):

    n = int( T / dt )

    cf.destination[:] = des

    for _ in range(n):
        P_pos = des[3:] - posvel[:3]
        D_pos = des[3:] - posvel[3:]

        acccmd[:] = P_pos * Kp + D_pos * Kd + [0,0,9.81]

        sleep(dt)