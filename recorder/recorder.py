from threading import Thread

from numpy import zeros, array

from time  import sleep

from .visualizer import *



class Recorder(Thread):


    def __init__(self, cf, n=2000):
        ## for threading
        super().__init__(daemon=True)

        self.record_length = 0
        self.recording     = True

        ## callback functions
        self.record_callback = {
            'acc'   : array_type_data_callback,
            'vel'   : array_type_data_callback,
            'pos'   : array_type_data_callback,
            'cmd'   : array_type_data_callback,
            'att'   : array_type_data_callback,
            'thrust': float_type_data_callback
        }
        ## data storage
        self.record_datastrg = {
            'acc'   : zeros(3,n),
            'vel'   : zeros(3,n),
            'pos'   : zeros(3,n),
            'cmd'   : zeros(3,n),
            'att'   : zeros(3,n),
            'thrust': zeros(1,n)
        }
        ## realtime data
        self.realtime_data = {
            'acc'   : cf.acc,
            'vel'   : cf.posvel[3:],
            'pos'   : cf.posvel[:3],
            'cmd'   : cf.command,
            'att'   : cf.euler_pos,
            'thrust': 0
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
        cmd    = self.record_datastrg['cmd']
        att    = self.record_datastrg['att']
        thrust = self.record_datastrg['thrust']

        plot_acc_pos_cmd(acc, pos, cmd, _len)
        plot_thrust(thrust, _len)
        plot_vel(vel, _len)


def array_type_data_callback(datastrg, data, i):

    datastrg[:,i] = array(data)


def float_type_data_callback(datastrg, data, i):

    datastrg[i] = int(data)