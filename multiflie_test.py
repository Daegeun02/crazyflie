import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

import numpy as np
import time

from sensor import start
from sensor import QtmWrapper

from takeoff_test import takeoff

# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E704')
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E709')

# The name of the rigid body in QTM that represents the Crazyflie
rigid_body_name1 = 'cf1'
rigid_body_name2 = 'cf2'


if __name__ == "__main__":

    cflib.crtp.init_drivers()

    qtm_wrapper1 = QtmWrapper(body_name=rigid_body_name1)

    ## crazyflie
    scf = SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache'))

    ## start sensor
    start(scf, qtm_wrapper1)

    ## rest
    time.sleep(1)

    ## takeoff
    takeoff(scf.cf)

    qtm_wrapper1.close()