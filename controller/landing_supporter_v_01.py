## landing supporter
from numpy import zeros, array

from time import sleep

## read constants
from constants import read_constant

_const = read_constant('gain')
Kp     = _const["Kp"]
Kd     = _const["Kd"]

_const = read_constant('gravity')
g      = _const["g"]


## extern variable
acccmd = zeros(3)
P_pos  = zeros(3)
D_pos  = zeros(3)
careg  = array([0,0,g])



def landing_supporter(cf, des, commander, T=2, dt=0.1, step=0.075):

    print('landing supporter on')

    n = int( T / dt )

    posvel = cf.posvel

    for _ in range(n):
        P_pos[:] = des[:3] - posvel[:3]
        D_pos[:] = des[3:] - posvel[3:]

        careg[2] -= step

        acccmd[:] = 0
        acccmd[:] += P_pos * Kp
        acccmd[:] += D_pos * Kd
        acccmd[:] += careg

        cf.command[:] = acccmd

        sleep(dt)

    print('landing')

    commander.stop_send_setpoint()