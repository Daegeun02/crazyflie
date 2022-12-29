from numpy.linalg import norm as norm
from numpy import zeros, array
from time import sleep

import copy

from .acc_att_controller import _dot_thrust, _dot_thrust2, alpha, Kp

from filter import LPF
from filter import EMAF

class Commander:
    
    band_limit = 10

    def __init__(self, cf, qtm, dt):
        self.cf  = cf
        self.qtm = qtm
        self.lpf = LPF(band_limit=self.__class__.band_limit, dt=dt)._filter
        self.emf = EMAF()._filter
        self.dt  = dt

        self.thrust = 10001
        self.record1 = []
        self.record2 = []

        self.acc_pre = [0,0,9.8]
    
    def init_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## initialize
        commander.send_setpoint(0, 0, 0, 0)


    def send_setpoint(self, cmd, n=10):
        ## crazyflie
        cf = self.cf
        ## commander
        commander = cf.commander
        ## to debug
        record1 = self.record1
        record2 = self.record2
        ## timestep
        dt = self.dt / n
        ## filter
        _filter = self.emf

        ## previous
        acc_pre = self.acc_pre

        ## current
        acc_cur = cf.acc

        ## controller input
        r_cmd, p_cmd, y_cmd, acc_cmd = cmd

        for _ in range(n):
            ## current state
            acc_cur = _filter(cf.acc)

            ## closed loop
            # self.thrust += _dot_thrust(acc_cmd, acc_cur)       ## integration
            self.thrust += _dot_thrust2(acc_cmd, acc_cur, acc_pre, dt)       ## integration

            ## cliping
            thrust = thrust_clip(self.thrust)

            record1.append(acc_cur[2]) 
            record2.append(thrust)

            ## input
            # commander.send_setpoint(r_cmd, p_cmd, y_cmd, thrust)

            acc_pre = array(acc_cur)

            sleep(dt)

        self.acc_pre = array(acc_cur)


    def stop_setpoint(self):
        commander = self.cf.commander

        commander.send_stop_setpoint()


def thrust_clip(thrust_cmd):
    ## thrust clip thrust
    if (thrust_cmd > 60000):
        thrust_cmd = 60000
    elif (thrust_cmd < 10001):
        thrust_cmd = 10001

    return int(thrust_cmd)