from numpy.linalg import norm

## thrust factor constant
alpha = (45000/9.81)
## P loop constant
Kp = 0.3 

K_T = alpha * Kp



def _dot_thrust(command, acc_cur):
    ## difference
    dot_acc = command[3] - norm( acc_cur )
    ## command
    dot_thr = K_T * dot_acc
    ## return
    return dot_thr


def _thrust_clip(thrust_cmd):
    ## thrust clip thrust
    if (thrust_cmd > 60000):
        thrust_cmd = 60000
    elif (thrust_cmd < 10001):
        thrust_cmd = 10001

    return int(thrust_cmd)