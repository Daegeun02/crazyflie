from threading import Thread

from time import sleep

from .cmd_generator import acccmd, generate_acccmd



class Tracker(Thread):


    def __init__(self, cf, dt):
        ## for threading
        super().__init__()
        self.daemon = True

        ## crazyflie
        self.cf = cf
        ## time step
        self.dt = dt

        ## trajectory
        self.traj = None

        self.tracking = False

    
    def run(self):
        ## crazyflie
        cf = self.cf
        ## time step
        T  = self.dt

        ## trajectory
        traj = self.traj

        print('waiting for trajectory')

        while not self.tracking:
            sleep(0.1)

        print('trajectory is uploaded, tracking start')

        while self.tracking:

            generate_acccmd(cf, traj, dt=T)

            cf.command[:] = acccmd

            sleep(T)


    def upload_traj(self):
        ## when trajectory uploaded start tracking
        self.tracking = True


    def update_traj(self):
        
        pass


    def stop_tracking(self):

        self.tracking = False