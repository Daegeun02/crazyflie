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
            'cmd'   : array_type_data_callback,
            'att'   : array_type_data_callback,
            'attimu': array_type_data_callback,
            'attcmd': array_type_data_callback,
            'thrust': float_type_data_callback
        }
        ## data storage
        self.record_datastrg = {
            'acc'   : zeros((3,n)),
            'vel'   : zeros((3,n)),
            'pos'   : zeros((3,n)),
            'cmd'   : zeros((3,n)),
            'att'   : zeros((3,n)),
            'attimu': zeros((3,n)),
            'attcmd': zeros((4,n)),
            'thrust': zeros((1,n))
        }
        ## realtime data
        self.realtime_data = {
            'acc'   : cf.acc,
            'vel'   : cf.posvel[3:],
            'pos'   : cf.posvel[:3],
            'cmd'   : cf.command,
            'att'   : cf.euler_pos,
            'attimu': cf.euler_pos_imu,
            'attcmd': commander.command,
            'thrust': commander.thrust
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


def array_type_data_callback(datastrg, data, i):

    datastrg[:,i] = array(data)


def float_type_data_callback(datastrg, data, i):

    datastrg[0,i] = int(data)