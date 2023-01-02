from numpy.linalg import norm

## thrust factor constant
alpha = (45000/9.81)
## P loop constant
Kp = 0.3 



def _dot_acceleration(acc_cmd, acc_cur):
    ## difference
    dot_acc = acc_cmd - acc_cur
    ## return
    return dot_acc              ## vector with shape (3,)


def _dot_thrust(acc_cmd, acc_cur):
    ## difference
    dot_thr = norm(acc_cmd - acc_cur)
    ## command
    dot_thr = Kp * alpha * dot_thr
    ## return
    return dot_thr


def _thrust_clip(thrust_cmd):
    ## thrust clip thrust
    if (thrust_cmd > 60000):
        thrust_cmd = 60000
    elif (thrust_cmd < 10001):
        thrust_cmd = 10001

    return int(thrust_cmd)