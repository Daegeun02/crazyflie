## high level commander
## support takeoff, hover, landing automatically and so on

from .integral_loop import smooth_command

from .landing_supporter import landing_supporter

from numpy        import array, arange
from numpy.linalg import norm

from time import sleep

## read constants
from constants import read_constant

_const = read_constant('gain') 
Kp     = _const["Kp"]
Kd     = _const["Kd"]

Kp = array([4.000,4.000,2.000])
Kd = array([3.200,3.200,2.400])



def takeoff(scf, commander, T=3, dt=0.1):

    print('takeoff')

    cf = scf.cf

    commander.init_send_setpoint()

    T = arange(0, T, dt)
    n = len(T)

    posvel = cf.posvel

    cur = array(posvel[:3])
    des = array([cur[0],cur[1],1.5])

    cf.destination[:3] = des

    for k in range(n):
        
        pos_cmd = smooth_command(des, cur, T[k], 2.0)
        P_pos   = pos_cmd - posvel[:3]
        D_pos   = posvel[3:]

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        cf.command[:] = acc_cmd

        sleep(dt)

    
def hover(scf, commander, T, dt=0.1):

    print('hover')

    cf = scf.cf

    T = arange(0, T, dt)
    n = len(T)

    posvel = cf.posvel
    acc    = cf.acc

    des = array(posvel[:3])

    cf.destination[:3] = des

    for _ in range(n):
        P_pos = des - posvel[:3]
        D_pos = posvel[3:]

        print(norm(acc))

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        cf.command[:] = acc_cmd

        sleep(dt)

    
def goto(scf, des, commander, T=4, dt=0.1):

    print('goto')

    cf = scf.cf

    T = arange(0, T, dt)
    n = len(T)

    posvel = cf.posvel

    cur = array(posvel[:3])

    cf.destination[:3] = des

    for k in range(n):
        pos_cmd = smooth_command(des, cur, T[k], 2)
        P_pos = pos_cmd - posvel[:3]
        D_pos = posvel[3:]

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        cf.command[:] = acc_cmd

        sleep(dt)


def landing(scf, commander, T=3, dt=0.1):

    print('landing')

    cf = scf.cf

    T = arange(0, T, dt)
    n = len(T)

    posvel = cf.posvel

    cur = array(posvel[:3])
    des = array([cur[0],cur[1],0])

    cf.destination[:3] = des

    for k in range(n):
        pos_cmd = smooth_command(des, cur, T[k], T[-1])
        P_pos   = pos_cmd - posvel[:3]
        D_pos   = posvel[3:]

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        cf.command[:] = acc_cmd

        if norm(posvel[:3] - des) < 0.20:
            break

        sleep(dt)

    landing_supporter(cf, commander)

    commander.stop_send_setpoint()