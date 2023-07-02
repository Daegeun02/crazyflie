from cflib.crazyflie.log import LogConfig

from .imu_setup import *



period_in_ms = 10


class IMU:

    def __init__( self, scf ):
        self.scf = scf
        self.cf  = scf.cf

        adjust_orientation_sensitivity( self.cf )
        activate_kalman_estimator( self.cf )
        reset_estimator( self.cf )


    def start_get_vel(self, period_in_ms=period_in_ms):
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


    def velocity_callback(self, timestamp, data, logconf):
        self.cf.vel[0] = data['stateEstimate.vx']
        self.cf.vel[1] = data['stateEstimate.vy']
        self.cf.vel[2] = data['stateEstimate.vz']


    def accelerate_callback(self, timestamp, data, logconf):
        self.cf.acc[0] = data['acc.x'] * 9.81
        self.cf.acc[1] = data['acc.y'] * 9.81
        self.cf.acc[2] = data['acc.z'] * 9.81