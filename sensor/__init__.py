## sensor package
from numpy import zeros, eye, ones

from .sensor_setup           import *
from .imu                    import IMU
from .qualisys_estimator     import QtmWrapper, SendPose



def setup(cf):
    ## zzzzzzzzzz
    adjust_orientation_sensitivity(cf)
    activate_kalman_estimator(cf)
    reset_estimator(cf)

    ## memory space
    cf.posvel    = zeros(6)
    cf.euler_pos = zeros(3)

    cf.acc = zeros(3)

    cf.command = zeros(3)

    cf.destination = zeros(3)

    cf.rot = eye(3)


def start(scf, qtm_wrapper):
    ## crazyflie
    cf = scf.cf

    ## setup sensors
    setup(cf)

    ## IMU
    imu = IMU(scf)
    imu.start_get_acc()
    imu.start_get_vel()

    ## qualisys beacon
    qtm_wrapper.on_pose = lambda pose: SendPose.send_extpose( cf, pose )