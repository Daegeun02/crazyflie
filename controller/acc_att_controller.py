from numpy.linalg import norm

from time import time

## thrust factor constant
alpha = (45000/9.81)
## P loop constant
Kp = 0.1 

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


def get_velocity():
    pass

class GetVelocity:

    def __init__(self):
        self.t = -1
        pass

    def get_velocity(self):
        pre_t = self.t
        cur_t = time()
        dt = cur_t - pre_t
        
        pass
    pass