## sensor package

from .sensor_setup           import *
from .imu                    import IMU
from .qualisys_estimator     import QtmWrapper, send_extpose_rot_matrix



def setup(cf):
    ## zzzzzzzzzz
    adjust_orientation_sensitivity(cf)
    activate_kalman_estimator(cf)
    reset_estimator(cf)


def start(scf):
    cf = scf.cf
    ## sensor setup
    setup(cf)
    ## initialize sensor
    sensor = IMU(scf=scf)
    ## start sensor
    sensor.start_get_position()
    sensor.start_get_velocity()
    sensor.start_get_acc()
    sensor.start_get_euler()


# def start(scf, qtm_wrapper):
#     cf = scf.cf
#     ## initialize sensor
#     # qtm_wrapper = QtmWrapper(scf.body_name)
#     sensor      = IMU(scf=scf)
#     ## setup sensor
#     qtm_wrapper.on_pose = lambda pose: send_extpose_rot_matrix(
#         cf, pose[0], pose[1], pose[2], pose[3]
#     )                   ## activate Qualisys bacon zz
#     setup(cf)           ## activate IMU
#     ## start sensor
#     ## translation
#     sensor.start_get_position()
#     sensor.start_get_velocity()
#     sensor.start_get_acc()
#     ## rotation
#     sensor.start_get_euler()