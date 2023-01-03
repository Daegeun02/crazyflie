## sensor package
from numpy import zeros

from .sensor_setup           import *
from .imu                    import IMU
from .qualisys_estimator     import QtmWrapper, send_pose



def setup(cf):
    ## zzzzzzzzzz
    adjust_orientation_sensitivity(cf)
    activate_kalman_estimator(cf)
    reset_estimator(cf)

    ## memory space
    cf.pos = zeros(3)
    cf.vel = zeros(3)
    cf.acc = zeros(3)

    cf.euler_pos = zeros(3)


def start(scf, qtm_wrapper):
    ## crazyflie
    cf = scf.cf

    ## setup sensors
    setup(cf)

    ## IMU
    imu = IMU(scf)
    imu.start_get_vel()
    imu.start_get_acc()

    ## qualisys beacon
    qtm_wrapper.on_pose = lambda pose: send_pose(
        cf, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
    )