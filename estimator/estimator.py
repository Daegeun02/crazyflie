## Threading
from threading import Thread

from numpy import zeros

from time import sleep



class Estimator(Thread):

    def __init__(self, scf, dt=0.1):
        ## for threading
        super().__init__()

        ## crazyflie
        self.cf = scf.cf
        ## time step
        self.dt = dt

        self.estimate_length = 0
        self.estimating      = True

        ## estimated position and velocity
        self.posvel = zeros(6)

    def run(self):
        dt     = self.dt
        acc    = self.cf.command
        posvel = self.posvel

        A, B, G = self.__class__._generate_dynamic_matrix(dt)

        sleep(dt)

        while self.estimating:

            posvel[:] = (A @ posvel + B @ acc + G).reshape(6,)

            self.estimate_length += 1

            sleep(dt)

    def stop_estimate(self):

        self.estimating = False

    @classmethod
    def _generate_dynamic_matrix(cls, dt, g=9.81):

        A = zeros((6,6))        ## state transition matrix
        B = zeros((6,3))        ## input matrix
        G = zeros((6,1))        ## gravity matrix

        A[0,0] = 1
        A[1,1] = 1
        A[2,2] = 1
        A[3,3] = 1
        A[4,4] = 1
        A[5,5] = 1
        A[0,3] = dt
        A[1,4] = dt
        A[2,5] = dt

        B[0,0] = dt
        B[1,1] = dt
        B[2,2] = dt
        B[3,0] = 0.5 * dt * dt
        B[4,1] = 0.5 * dt * dt
        B[5,2] = 0.5 * dt * dt

        G[2,0] = dt * (-9.81)
        G[5,0] = 0.5 * dt * dt * (-9.81)

        return A, B, G