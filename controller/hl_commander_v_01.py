## high level commander
## support takeoff, hover, landing automatically and so on

from .acc_att_controller import smooth_command

from numpy        import array, arange
from numpy.linalg import norm

from time import sleep

Kp = array([4.000,4.000,2.000])
Kd = array([2.700,2.700,2.100])



def takeoff(scf, commander, T=3, dt=0.1):

    cf = scf.cf

    commander.init_send_setpoint()

    T = arange(0, T, dt)
    n = len(T)

    posvel = cf.posvel

    cur = array(posvel[:3])
    des = array([cur[0],cur[1],1.5])

    for k in range(n):
        
        pos_cmd = smooth_command(des, cur, T[k], 2)
        P_pos   = pos_cmd - posvel[:3]
        D_pos   = posvel[3:]

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        cf.command[:] = acc_cmd

        sleep(0.1)

    
def hover(scf, commander, T, dt=0.1):

    cf = scf.cf

    T = arange(0, T, dt)
    n = len(T)

    posvel = cf.posvel

    des = array(posvel[:3])

    for k in range(n):
        P_pos = des - posvel[:3]
        D_pos = posvel[3:]

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        cf.command[:] = acc_cmd

        sleep(0.1)


def landing(scf, commander, T=3, dt=0.1):

    cf = scf.cf

    T = arange(0, T, dt)
    n = len(T)

    posvel = cf.posvel

    cur = array(posvel[:3])
    des = array([cur[0],cur[1],0])

    for k in range(n):
        pos_cmd = smooth_command(des, cur, T[k], 2)
        P_pos   = pos_cmd - posvel[:3]
        D_pos   = posvel[3:]

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        cf.command[:] = acc_cmd

        if norm(posvel[:3] - des) < 0.05:
            print('fine landing')
            break

        sleep(0.1)

    commander.stop_send_setpoint()