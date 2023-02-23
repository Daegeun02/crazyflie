import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from time import sleep

from sensor import start
from sensor import QtmWrapper

from controller import Commander
from controller import Commander_v_01
from controller import takeoff, hover, landing
from controller import takeoff_v_01, hover_v_01, landing_v_01

from recorder import Recorder
from recorder import plot_acc_pos_cmd, plot_thrust, plot_vel, plot_att

from admm_gpu_main import guidance_gpu_2

uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705')

rigid_body_name = 'cf1'


def test_flight_seq1(scf):

    ## test flight
    commander = Commander(scf, dt=0.1)

    takeoff(scf, commander)

    hover(scf, commander)

    landing(scf, commander)


def test_flight_seq2(scf):

    commander = Commander_v_01(scf, dt=0.1)

    commander.daemon = True

    recorder = Recorder(scf)
    
    recorder.daemon = True

    commander.start()

    recorder.start()

    takeoff_v_01(scf, commander)

    hover_v_01(scf, commander, T=3)

    landing_v_01(scf, commander)

    recorder.stop_record()

    commander.join()

    recorder.join()

    _len = recorder.record_length

    acc    = recorder.record_datastrg['acc']
    vel    = recorder.record_datastrg['vel']
    pos    = recorder.record_datastrg['pos']
    cmd    = recorder.record_datastrg['cmd']
    att    = recorder.record_datastrg['att']
    thrust = recorder.record_datastrg['thrust']

    plot_acc_pos_cmd(acc, pos, cmd, _len)
    plot_thrust(thrust, _len)




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

        commander = Commander_v_01(scf, dt=0.1)

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

        _len = recorder.record_length

        acc    = recorder.record_datastrg['acc']
        vel    = recorder.record_datastrg['vel']
        pos    = recorder.record_datastrg['pos']
        cmd    = recorder.record_datastrg['cmd']
        att    = recorder.record_datastrg['att']
        attimu = recorder.record_datastrg['attimu']
        attcmd = recorder.record_datastrg['attcmd']
        thrust = recorder.record_datastrg['thrust']

        plot_acc_pos_cmd(acc, pos, cmd, _len)
        plot_thrust(thrust, _len)
        plot_att(att, attimu, attcmd, _len)

    qtm_wrapper.close()