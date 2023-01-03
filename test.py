import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

import numpy as np
import time

from sensor   import start
from test_fly import test_fly, test_fly2
from visualizer import visualize_acc, visualize_state

from sensor import QtmWrapper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E704')

# The name of the rigid body in QTM that represents the Crazyflie
rigid_body_name = 'cf1'

def sensoring():
    t = np.linspace(0,5,501)

    dt = t[1] - t[0]

    ## sensoring debug
    for _ in range(len(t)):
        print(f"acc: {cf.acc}")
        print(f"vel: {cf.vel}")
        print(f"pos: {cf.pos}")

        print(f'euler_pos: {cf.euler_pos}')

        print('=' * 20)

        time.sleep(dt)


if __name__ == "__main__":

    t  = np.linspace(0,5,51)
    dt = t[1] - t[0]

    cflib.crtp.init_drivers()

    qtm_wrapper = QtmWrapper(body_name=rigid_body_name)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        ## sensor start
        start(scf, qtm_wrapper)

        ## wait for start up sensor
        time.sleep(1)

        ## control function
        # test_fly(t, cf, n=5)
        test_fly2(t, cf)

        ## sensoring debug
        # sensoring()

    qtm_wrapper.close()