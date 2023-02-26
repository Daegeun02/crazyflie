from threading import Thread

from time import sleep

from .cmd_generator import generate_acccmd



class Tracker(Thread):


    def __init__(self, cf, dt):
        ## for threading
        super().__init__()
        self.daemon = True

        ## crazyflie
        self.cf = cf
        ## time step
        self.dt = dt

        self.tracking = False

    
    def run(self):
        ## crazyflie
        cf = self.cf
        ## time step
        dt = self.dt

        print('waiting for trajectory')

        while not self.tracking:
            sleep(0.1)

        print('trajectory is uploaded, tracking start')

        while self.tracking:

            cf.command[:] = generate_acccmd()

            sleep(dt)


    def upload_traj(self):
        ## when trajectory uploaded start tracking
        self.tracking = True


    def update_traj(self):
        
        pass


    def stop_tracking(self):

        self.tracking = False