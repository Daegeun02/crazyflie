from numpy.linalg import norm

## thrust factor constant
alpha = (45000/9.81)
## PD loop constant
Kp = 0.3 
# Kd = 0.1

K_T = alpha * Kp



def _dot_thrust(acc_cmd, acc_cur):
    ## difference
    dot_acc = acc_cmd - acc_cur[2]      ## temporary
    # print('dot_acc', dot_acc)
    # print('acc_cur', acc_cur)
    ## command
    dot_thr = K_T * dot_acc
    ## return
    return dot_thr


# def _dot_thrust2(acc_cmd, acc_cur, acc_pre, dt):
#     print(dt)
#     ## difference
#     del_acc = acc_cmd    - acc_cur[2]      ## temporary
#     print('acc_cmd', acc_cmd)
#     print('acc_cur', acc_cur)
#     print('acc_pre', acc_pre)
#     print("del_acc", del_acc)
#     dot_acc = (acc_cur[2] - acc_pre[2]) / dt
#     print("dot_acc", dot_acc)

#     print('=' * 20)
#     ## command
#     dot_thr = 0
#     dot_thr += del_acc * Kp
#     dot_thr -= dot_acc * Kd
#     dot_thr *= alpha
#     ## return
#     return dot_thr


class AccAttController:

    K_ap = 0.707
    ## thrust factor constant
    alpha = (45000/9.81)
    ## PD loop gain
    Kp = 0.3
    Kd = 0.707

    def __init__(self, state, cf, qtm):
        
        self.cf = cf
        self.qtm = qtm

        self.state = state
        
        self.thrust_compansation = 0

    def NED2RPY(self, accNED):
        pass        


    def print_sensor(self):
        pass


    def send_setpoint(self, commander, roll_c, pitch_c, yaw_c, acc_c):

        """
        Control Input : Acceleration in NED frame (m/s^2)
        """        

        K_pt = 0.707
        K_py = 0.707

        F_T = 45500/9.8

        acc_m = 0 # to estimate from telem

        dt = 0.01

        yaw = 0

        T_c = 0

        for _ in range(10):

            # acc_m = interior
            # yaw = qualisys
            # yawrate = interior

            T_dot = F_T*K_pt*(acc_c - acc_m) 

            T_c = T_c + T_dot*dt

            yawrate = K_py*(yaw_c - yaw) 

            commander.send_setpoint( roll_c, pitch_c, yawrate, T_c )

            
