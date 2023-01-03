import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

import numpy as np
import time

from sensor   import start

from sensor import QtmWrapper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E704')

# The name of the rigid body in QTM that represents the Crazyflie
rigid_body_name = 'cf1'

def sensoring():
    t = np.linspace(0,5,51)

    dt = t[1] - t[0]

    acc_rec = np.zeros((3,len(t)))
    pos_rec = np.zeros((3,len(t)))

    euler_pos_rec = np.zeros((3,len(t)))

    ## sensoring debug
    for i in range(len(t)):
        acc_rec[:,i] = cf.acc
        pos_rec[:,i] = cf.pos

        euler_pos_rec[:,i] = cf.euler_pos

        print(f"acc: {cf.acc}")
        print(f"pos: {cf.pos}")

        print(f'euler_pos: {cf.euler_pos}')

        print('=' * 20)

        time.sleep(dt)

if __name__ == "__main__":

    cflib.crtp.init_drivers()

    qtm_wrapper = QtmWrapper(body_name=rigid_body_name)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        scf.body_name = rigid_body_name

        ## sensor start
        start(scf, qtm_wrapper)

        ## wait for start up sensor
        time.sleep(1)

        ## sensoring debug
        sensoring()

    qtm_wrapper.close()