## sensoring
import numpy as np

from cflib.crazyflie.log import LogConfig

period_in_ms = 10

class IMU:

    def __init__(self, scf):
        # self.state = state
        self.scf = scf
        self.cf  = scf.cf

        ## state
        self.cf.pos = np.zeros(3)
        self.cf.vel = np.zeros(3)
        self.cf.acc = np.zeros(3)

        self.cf.euler_pos = np.zeros(3)
        self.cf.euler_vel = np.zeros(3)


    def start_get_position(self, period_in_ms=period_in_ms):
        log_conf = LogConfig(name="position", period_in_ms=period_in_ms)
        log_conf.add_variable('kalman.stateX', 'FP16')      # m
        log_conf.add_variable('kalman.stateY', 'FP16')      # m
        log_conf.add_variable('kalman.stateZ', 'FP16')      # m

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()


    def start_get_velocity(self, period_in_ms=period_in_ms):
        log_conf = LogConfig(name="velocity", period_in_ms=period_in_ms)
        log_conf.add_variable('stateEstimate.vx', 'FP16')
        log_conf.add_variable('stateEstimate.vy', 'FP16')
        log_conf.add_variable('stateEstimate.vz', 'FP16')

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.velocity_callback)
        log_conf.start()
 

    def start_get_acc(self, period_in_ms=period_in_ms):
        log_conf = LogConfig(name='acceleration', period_in_ms=period_in_ms)
        log_conf.add_variable('acc.x', 'FP16')      ## m/s^2
        log_conf.add_variable('acc.y', 'FP16')      ## m/s^2
        log_conf.add_variable('acc.z', 'FP16')      ## m/s^2

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.accelerate_callback)
        log_conf.start()


    def start_get_euler(self, period_in_ms=period_in_ms):
        log_conf = LogConfig(name='Euler_angle', period_in_ms=period_in_ms)
        log_conf.add_variable('stateEstimate.roll' , 'FP16')
        log_conf.add_variable('stateEstimate.pitch', 'FP16')
        log_conf.add_variable('stateEstimate.yaw'  , 'FP16')

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.euler_callback)
        log_conf.start()


    def start_get_euler_vel(self, period_in_ms=period_in_ms):
        log_conf = LogConfig(name='Euler_angle_velocity', period_in_ms=period_in_ms)
        log_conf.add_variable('gyro.x', 'FP16')     ## deg/s
        log_conf.add_variable('gyro.y', 'FP16')     ## deg/s
        log_conf.add_variable('gyro.z', 'FP16')     ## deg/s

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.eulervel_callback)
        log_conf.start()

    
    ### callbacks ### 
    def position_callback(self, timestamp, data, logconf):
        self.cf.pos[0] = data['kalman.stateX']
        self.cf.pos[1] = data['kalman.stateY']
        self.cf.pos[2] = data['kalman.stateZ']


    def velocity_callback(self, timestamp, data, logconf):
        self.cf.vel[0] = data['stateEstimate.vx']
        self.cf.vel[1] = data['stateEstimate.vy']
        self.cf.vel[2] = data['stateEstimate.vz']


    def accelerate_callback(self, timestamp, data, logconf):
        self.cf.acc[0] = data['acc.x'] * 9.81
        self.cf.acc[1] = data['acc.y'] * 9.81
        self.cf.acc[2] = data['acc.z'] * 9.81

    
    def euler_callback(self, timestamp, data, logconf):
        self.cf.euler_pos[0] = data['stateEstimate.roll']
        self.cf.euler_pos[1] = data['stateEstimate.pitch']
        self.cf.euler_pos[2] = data['stateEstimate.yaw']


    def eulervel_callback(self, timestamp, data, logconf):
        self.cf.euler_vel[0] = data['gyro.x']
        self.cf.euler_vel[1] = data['gyro.y']
        self.cf.euler_vel[2] = data['gyro.z']
