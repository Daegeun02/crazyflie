from threading import Thread

from numpy import zeros, array

from time  import sleep

from .visualizer import *



class Recorder(Thread):


    def __init__(self, scf, commander, n=10000):
        ## for threading
        super().__init__()

        self.record_length = 0
        self.recording     = True

        cf = scf.cf

        ## callback functions
        self.record_callback = {
            'acc'   : array_type_data_callback,
            'vel'   : array_type_data_callback,
            'pos'   : array_type_data_callback,
            'acccmd': array_type_data_callback,
            'att'   : array_type_data_callback,
            'attimu': array_type_data_callback,
            'cmd'   : array_type_data_callback
        }
        ## data storage
        self.record_datastrg = {
            'acc'   : zeros((3,n)),
            'vel'   : zeros((3,n)),
            'pos'   : zeros((3,n)),
            'acccmd': zeros((3,n)),
            'att'   : zeros((3,n)),
            'attimu': zeros((3,n)),
            'cmd'   : zeros((4,n))
        }
        ## realtime data
        self.realtime_data = {
            'acc'   : cf.acc,
            'vel'   : cf.posvel[3:],
            'pos'   : cf.posvel[:3],
            'acccmd': cf.command,
            'att'   : cf.euler_pos,
            'attimu': cf.euler_pos_imu,
            'cmd'   : commander.command
        }

    
    def run(self):

        sleep(0.1)

        while self.recording:

            for key, callback in self.record_callback.items():

                datastrg = self.record_datastrg[key]
                data     = self.realtime_data[key]

                callback( datastrg, data, self.record_length )

            self.record_length += 1

            sleep(0.05)

    
    def stop_record(self):

        self.recording = False

    
    def join(self):

        super().join()

        _len = self.record_length

        acc    = self.record_datastrg['acc']
        vel    = self.record_datastrg['vel']
        pos    = self.record_datastrg['pos']
        acccmd = self.record_datastrg['acccmd']
        att    = self.record_datastrg['att']
        attimu = self.record_datastrg['attimu']
        cmd    = self.record_datastrg['cmd']

        plot_acc_pos_cmd(acc, pos, acccmd, _len)
        plot_thrust(cmd[3,:], _len)
        plot_att(att, attimu, cmd[:3,:], _len)


def array_type_data_callback(datastrg, data, i):

    datastrg[:,i] = array(data)


def float_type_data_callback(datastrg, data, i):

    datastrg[0,i] = int(data)