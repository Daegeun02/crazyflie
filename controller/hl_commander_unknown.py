## high level commander
## support takeoff, landing automatically

## numpy
from numpy        import array, zeros
from numpy.linalg import matrix_power

## solver
from scipy.sparse.linalg import lsqr

## commander
from .commander import Commander



def takeoff(cf, T=3, dt=0.1):
    ## steps
    n = int(T / dt)

    ## commander
    commander = Commander(cf, dt)
    commander.init_send_setpoint()

    print("ready to flight, takeoff")

    for k in range(n):
        acc_cmd = acc_cmds[:,k]

        commander.send_setpoint_ENU(acc_cmd)

    print("ready for next mission")

    return commander


def landing(cf, _else):
    pass


class MinimumEnergyControl:
    
    def __call__(self, cf, dt, n):
        ## state
        x_0 = zeros((6,1))                  ## initial state
        x_0[:3,0] = array(cf.pos)
        x_0[3:,0] = array(cf.vel)

        ## generate problem: Minimum Energy Control
        des = x_0[:3,0] + array([0,0,1])    ## destination

        G, C = self.generate_problem(x_0, des, dt)

        acc_cmds = lsqr(G, C)[0]            ## solve
        acc_cmds = acc_cmds.reshape(n,3).T

        return acc_cmds


    def generate_problem(self, x_0, des, dt, n):
        ## state matrices
        cls = self.__class__
        A   = cls.A
        B   = cls.B
        grv = cls.g

        g = zeros((6,6))
        G = zeros((6,3*n))
        
        for i in range(n):
            idx = 3 * i
            A_n = matrix_power(A, n-i-1)
            g[:,0] += A_n
            G[:,idx:idx+3] = A_n @ B

        g = g @ grv

        C = des - matrix_power(A, n) @ x_0[:3,0] - g    

        return G, C


def _takeoff(cf, dt, n):
    MinimumEnergyControl()(cf, dt, n)