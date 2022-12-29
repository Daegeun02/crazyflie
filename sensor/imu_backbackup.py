## sensoring
import numpy as np

from cflib.crazyflie.log import LogConfig



class IMU:

    def __init__(self, scf, state):
        self.state = state
        self.scf = scf
        self.cf  = scf.cf


    def start_get_position(self, period_in_ms=10):
        log_conf = LogConfig(name="position", period_in_ms=period_in_ms)
        log_conf.add_variable('kalman.stateX', 'FP16')      # m
        log_conf.add_variable('kalman.stateY', 'FP16')      # m
        log_conf.add_variable('kalman.stateZ', 'FP16')      # m

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()


    def start_get_velocity(self, period_in_ms=10):
        log_conf = LogConfig(name="velocity", period_in_ms=period_in_ms)
        log_conf.add_variable('kalman.statePX', 'FP16')     ## m/s
        log_conf.add_variable('kalman.statePY', 'FP16')     ## m/s
        log_conf.add_variable('kalman.statePZ', 'FP16')     ## m/s

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.velocity_callback)
        log_conf.start()
 

    def start_get_acc(self, period_in_ms=10):
        log_conf = LogConfig(name='accelerate', period_in_ms=period_in_ms)
        log_conf.add_variable('acc.x', 'FP16')      ## m/s^2
        log_conf.add_variable('acc.y', 'FP16')      ## m/s^2
        log_conf.add_variable('acc.z', 'FP16')      ## m/s^2

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.accelerate_callback)
        log_conf.start()


    def start_get_euler(self, period_in_ms=10):
        log_conf = LogConfig(name='Euler_ang', period_in_ms=period_in_ms)
        log_conf.add_variable('gyro.x', 'FP16')     ## deg/s
        log_conf.add_variable('gyro.y', 'FP16')     ## deg/s
        log_conf.add_variable('gyro.z', 'FP16')     ## deg/s

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.eulervel_callback)
        log_conf.start()


    ### """"""""""""""""" callbacks """"""""""""""""" ### 
    def position_callback(self, timestamp, data, logconf):
        self.state.pos[0] = data['kalman.stateX']
        self.state.pos[1] = data['kalman.stateY']
        self.state.pos[2] = data['kalman.stateZ']


    def velocity_callback(self, timestamp, data, logconf):
        self.state.vel[0] = data['kalman.statePX']
        self.state.vel[1] = data['kalman.statePY']
        self.state.vel[2] = data['kalman.statePZ']


    def accelerate_callback(self, timestamp, data, logconf):
        self.state.acc[0] = data['acc.x'] * 9.81
        self.state.acc[1] = data['acc.y'] * 9.81
        self.state.acc[2] = data['acc.z'] * 9.81


    def eulervel_callback(self, timestamp, data, logconf):
        self.state.angvel[0] = data['gyro.x']
        self.state.angvel[1] = data['gyro.y']
        self.state.angvel[2] = data['gyro.z']
