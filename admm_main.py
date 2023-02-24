import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from time import sleep

from numpy import array

from sensor import start
from sensor import QtmWrapper

from controller import Commander
from controller import takeoff, hover, landing, goto

from recorder import Recorder

from estimator import Estimator

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

    commander = Commander(scf, dt=0.1)      ## define commander object
    commander.daemon = True                 ## it must stop when guidance is done

    recorder = Recorder(scf, commander)     ## record flight data
    recorder.daemon = True                  ## it must stop when guidance is done

    estimator = Estimator(scf)              ## estimate ideal posvel
    estimator.daemon = True                 ## it must stop when guidance is done

    commander.start()                       ## thread start
    recorder.start()                        ## thread start
    estimator.start()                       ## thread start

    takeoff(scf, commander)

    hover(scf, commander, T=3)

    goto(scf, array([1,1,1.5]), commander)

    landing(scf, commander)

    recorder.stop_record()

    estimator.stop_estimate()

    commander.join()

    sleep(1)

    recorder.join()

    estimator.join()



if __name__ == "__main__":

    cflib.crtp.init_drivers()

    qtm_wrapper = QtmWrapper(body_name=rigid_body_name)

    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf:

        ## sensor start
        start(scf, qtm_wrapper)

        ## wait for start up sensor
        sleep(1)

        # test_flight_seq1(scf)

        # test_flight_seq2(scf)

        commander = Commander(scf, dt=0.1)

        commander.daemon = True

        recorder = Recorder(scf, commander)
        
        recorder.daemon = True

        commander.start()

        recorder.start()

        guidance_gpu_2(scf, scf.cf, commander)

        commander.join()

        sleep(1)
        
        recorder.stop_record()

        recorder.join()

    qtm_wrapper.close()