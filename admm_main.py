import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

import time

from sensor import start
from sensor import QtmWrapper

from controller import Commander
from controller import Commander_v_01
from controller import takeoff, hover, landing

from admm_gpu_main import guidance_gpu_2

uri1 = uri_helper.uri_from_env(default='radio://0/65/2M/E7E7E7E707')

rigid_body_name = 'cf1'


def test_flight_seq1(scf):

    ## test flight
    commander = Commander(scf, dt=0.1)

    takeoff(scf, commander)

    hover(scf, commander)

    landing(scf, commander)


def test_flight_seq2(scf):

    commander = Commander_v_01(scf, dt=0.1)

    commander.start()

    takeoff(scf, commander)

    hover(scf, commander)

    landing(scf, commander)

    commander.join()




if __name__ == "__main__":

    cflib.crtp.init_drivers()

    qtm_wrapper = QtmWrapper(body_name=rigid_body_name)

    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf:

        ## sensor start
        start(scf, qtm_wrapper)

        ## wait for start up sensor
        time.sleep(1)

        test_flight_seq1(scf)

        # test_flight_seq2(scf)

        # guidance_gpu_2(scf, scf.cf, commander)

    qtm_wrapper.close()