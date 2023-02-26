from threading import Thread

from time import sleep



class Tracker(Thread):

    def __init__(self):
        ## for threading
        super().__init__()
        self.daemon = True

        self.tracking = False

    
    def run(self):

        print('waiting for trajectory')

        while not self.tracking:
            sleep(0.1)

        print('trajectory is uploaded')
        print('tracking start')

        while self.tracking:

            pass


    def upload_traj(self):
        ## when trajectory uploaded start tracking
        self.tracking = True


    def update_traj(self):
        
        pass


    def stop_tracking(self):

        self.tracking = False