## high level commander
## support takeoff, hover, landing automatically and so on

from .integral_loop import smooth_command

from .landing_supporter import landing_supporter

from numpy        import array, zeros
from numpy.linalg import norm

from time import sleep

## read constants
from constants import read_constant

_const = read_constant('gain') 
Kp     = _const["Kp"]
Kd     = _const["Kd"]

_const = read_constant('gravity')
g      = _const["g"]


## extern variable
cur    = zeros(6)
des    = zeros(6)
descmd = zeros(6)
acccmd = zeros(3)
P_pos  = zeros(3)
D_pos  = zeros(3)
careg  = array([0,0,g])



def takeoff(cf, commander, h=1.5, T=3, dt=0.1):

    print('takeoff')

    commander.init_send_setpoint()

    n = int( T / dt )
    t = 0

    cur[:] = cf.posvel
    posvel = cf.posvel

    ## takeoff straight up
    des[0]  = cur[0]
    des[1]  = cur[1]
    des[2]  = h
    des[3:] = 0

    cf.destination[:] = des

    for _ in range(n):
        ## one way to delete overshoot
        descmd[:] = smooth_command(des, cur, t, int(T/2))

        PD_loop( descmd, posvel )

        cf.command[:] = acccmd

        t += dt

        sleep(dt)

        
def hover(cf, commander, T, dt=0.1):

    print('hover')

    n = int( T / dt )

    cur[:] = cf.posvel
    posvel = cf.posvel

    ## hold on at this position
    des[:3] = cur[:3]
    des[3:] = 0

    cf.destination[:] = des

    for _ in range(n):

        PD_loop( des, posvel )

        cf.command[:] = acccmd

        sleep(dt)


def goto(cf, des, commander, T=4, dt=0.1):

    print('goto')

    n = int( T / dt )
    t = 0

    cur[:] = cf.posvel
    posvel = cf.posvel

    cf.destination[:] = des

    for _ in range(n):
        ## one way to delete overshoot
        descmd[:] = smooth_command(des, cur, t, int(T/2))

        PD_loop( descmd, posvel )

        cf.command[:] = acccmd

        t += dt

        sleep(dt)


def landing(cf, commander, h=0.2, T=3, dt=0.1):

    print('landing')

    n = int( T / dt )
    t = 0

    cur[:] = cf.posvel
    posvel = cf.posvel

    ## landing straight down
    des[0]  = cur[0]
    des[1]  = cur[1]
    des[2]  = h
    des[3:] = 0

    cf.destination[:] = des

    for _ in range(n):
        ## one way to delete overshoot
        descmd[:] = smooth_command(des, cur, t, T)

        PD_loop( descmd, posvel )

        cf.command[:] = acccmd

        if norm(posvel - des) < 0.1:
            break

        t += dt

        sleep(dt)

    landing_supporter(cf, commander)

    commander.stop_send_setpoint()

    
def PD_loop(descmd, posvel):

    P_pos[:] = descmd[:3] - posvel[:3]
    D_pos[:] = descmd[3:] - posvel[3:]

    acccmd[:] = 0
    acccmd[:] += P_pos * Kp
    acccmd[:] += D_pos * Kd
    acccmd[:] += careg