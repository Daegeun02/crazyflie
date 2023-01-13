## high level commander
## support takeoff, landing automatically and so on

from .acc_att_controller import smooth_command

from numpy        import array, arange, clip
from numpy.linalg import norm

Kp = array([4.000,4.000,2.000])
Kd = array([2.700,2.700,2.100])



def takeoff(scf, commander, T=3, dt=0.1):

    commander.init_send_setpoint()

    T = arange(0,T,dt)                  ## timestamp
    n = len(T)                          ## steps

    pos = scf.cf.pos                    ## pointer
    vel = scf.cf.vel

    cur = array(pos)                    ## desired state
    des = array([cur[0],cur[1],1])      ## current state

    for k in range(n):
        pos_cmd = smooth_command(des, cur, T[k], 2)
        P_pos   = pos_cmd - pos
        D_pos   = vel

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        commander.send_setpoint_ENU( acc_cmd )


def landing(scf, commander, T=3, dt=0.1):

    T = arange(0,T,dt)                  ## timestamp
    n = len(T)                          ## steps

    pos = scf.cf.pos                    ## pointer
    vel = scf.cf.vel

    cur = array(pos)                    ## desired state
    des = array([cur[0],cur[1],0])      ## current state

    for k in range(n):
        pos_cmd = smooth_command(des, cur, T[k], 2)
        P_pos   = pos_cmd - pos
        D_pos   = vel

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,9.81]

        print(pos-des)

        commander.send_setpoint_ENU( acc_cmd )

        if norm(pos-des) < 0.05:
            print('fine landing')
            break

    commander.stop_send_setpoint()