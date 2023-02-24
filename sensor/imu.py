## sensoring
from cflib.crazyflie.log import LogConfig

period_in_ms = 10

class IMU:

    def __init__(self, scf):
        # self.state = state
        self.scf = scf
        self.cf  = scf.cf

    
    def start_get_pos(self, period_in_ms=period_in_ms):
        log_conf = LogConfig(name='position', period_in_ms=period_in_ms)
        log_conf.add_variable('kalman.stateX', 'FP16')
        log_conf.add_variable('kalman.stateY', 'FP16')
        log_conf.add_variable('kalman.stateZ', 'FP16')

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()


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


    def start_get_euler_vel(self, period_in_ms=period_in_ms):
        log_conf = LogConfig(name='Euler_angle_velocity', period_in_ms=period_in_ms)
        log_conf.add_variable('stateEstimate.roll', 'FP16')     ## deg/s
        log_conf.add_variable('stateEstimate.pitch', 'FP16')     ## deg/s
        log_conf.add_variable('stateEstimate.yaw', 'FP16')     ## deg/s

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.eulervel_callback)
        log_conf.start()

    
    ### callbacks ### 
    def position_callback(self, timestamp, data, logconf):
        self.cf.posvel_imu[0] = data['kalman.stateX']
        self.cf.posvel_imu[1] = data['kalman.stateY']
        self.cf.posvel_imu[2] = data['kalman.stateZ']

        
    def velocity_callback(self, timestamp, data, logconf):
        self.cf.posvel_imu[3] = data['stateEstimate.vx']
        self.cf.posvel_imu[4] = data['stateEstimate.vy']
        self.cf.posvel_imu[5] = data['stateEstimate.vz']


    def accelerate_callback(self, timestamp, data, logconf):
        self.cf.acc[0] = data['acc.x'] * 9.81
        self.cf.acc[1] = data['acc.y'] * 9.81
        self.cf.acc[2] = data['acc.z'] * 9.81

    
    def eulervel_callback(self, timestamp, data, logconf):
        self.cf.euler_pos_imu[0] = data['stateEstimate.roll']
        self.cf.euler_pos_imu[1] = data['stateEstimate.pitch']
        self.cf.euler_pos_imu[2] = data['stateEstimate.yaw']