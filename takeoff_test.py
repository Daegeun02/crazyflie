from numpy        import array, linspace, zeros, clip
from numpy.linalg import norm

from controller import Commander

from visualizer import visualize_flight, visualize_acc, visualize_acc_norm

Kp = array([4.000, 4.000, 2.000])
Kd = array([2.700, 2.700, 2.100])



def takeoff(scf, destination=[0,0,1], g=9.81, tol=1e-1):
    cf = scf.cf

    ## timestep
    T = linspace(0,5,51)
    dt = T[1] - T[0]
    
    ## commander
    commander = Commander(scf, dt)
    commander.init_send_setpoint()

    pos_rec = zeros((3,153))
    pos_ref = zeros((3,153))

    ## state
    pos   = cf.pos
    vel   = cf.vel

    ## loop
    ## takeoff
    print("takeoff")
    destination = pos + array([0,0,1])
    obj         = destination
    cur         = array(pos)
    for i in range(len(T)):
        pos_cmd = smooth_cmd(obj, cur, T[i])
        P_pos   = pos_cmd - pos
        D_pos   = vel
        
        # PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,g]
        acc_cmd[:2] = clip(acc_cmd[:2], -2, 2)

        ## command
        commander.send_setpoint_ENU(acc_cmd)

        pos_rec[:,i] = array(pos)
        pos_ref[:,i] = pos_cmd

    print("moving")
    destination = array([2,2,1])
    obj         = destination
    cur         = array(pos)
    for i in range(len(T)):
        pos_cmd = smooth_cmd(obj, cur, T[i])
        P_pos   = pos_cmd - pos
        D_pos   = vel

        # PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,g]
        acc_cmd[:2] = clip(acc_cmd[:2], -2, 2)

        ## command
        commander.send_setpoint_ENU(acc_cmd)

        pos_rec[:,i+51] = array(pos)
        pos_ref[:,i+51] = pos_cmd

    print("landing")
    destination = array([0,0,0])
    obj         = destination
    cur         = array(pos)
    for i in range(len(T)):
        pos_cmd = smooth_cmd(obj, cur, T[i])
        P_pos   = pos_cmd - pos
        D_pos   = vel

        ## PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd * 1.2
        acc_cmd += [0,0,g]
        acc_cmd = clip(acc_cmd, [-2,-2, 9], [ 2, 2,11])

        ## commmand
        commander.send_setpoint_ENU(acc_cmd)

        pos_rec[:,i+102] = array(pos)
        pos_ref[:,i+102] = pos_cmd

        if norm(pos-destination) < 0.05:
            print('fine landing')
            break

    print("land", i)

    commander.stop_send_setpoint()

    t2 = linspace(0,15,153)

    visualize_acc(pos_rec, pos_ref, t2)


def smooth_cmd(obj, cur, t, T=2):
    if t < T/2:
        return f1(obj, cur, t, T)
    elif t < T:
        return f2(obj, cur, t, T)
    else:
        return obj


def f1(obj, cur, t, T):
    out = (2/T**2) * (obj - cur) * t**2 + cur

    return out

def f2(obj, cur, t, T):
    out = (2/T**2) * (cur - obj) * (t-T)**2 + obj

    return out