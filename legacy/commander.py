## define Hz of loop
from time import sleep, time


## memory space
from numpy import zeros
from numpy.linalg import norm


## control loop
from ..controller.integral_loop import _dot_thrust
from ..controller.integral_loop import _thrust_clip, alpha


## transformer
from ..controller.optimus_prime import _command_as_ENU, _command_as_RPY
from ..controller.optimus_prime import _command_is_not_in_there



class Commander:
    

    def __init__(self, scf, dt):
        self.cf = scf.cf
        self.dt = dt

        self.thrust = alpha * 9.81
        ## store commands
        self.command = zeros(4)         ## RPY,T

        self.ready_for_command = False
    
    
    def init_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## initialize
        commander.send_setpoint(0, 0, 0, 0)
        self.ready_for_command = True


    ## command should be given in ENU
    def send_setpoint_ENU(self, acc_cmd, n=5):
        ## crazyflie
        cf = self.cf
        ## commander
        commander = cf.commander
        command   = self.command
        ## timestep
        dt = self.dt / n - 0.005
        ## acceleration current
        euler_cur = cf.euler_pos
        acc_cur   = cf.acc

        ## transform command
        acc_cmd = _command_is_not_in_there( euler_cur, acc_cmd )
        _command_as_RPY( acc_cmd, command )

        for _ in range(n):

            ## closed loop
            self.thrust += _dot_thrust( command, acc_cur )
            
            ## cliping
            thrust = _thrust_clip( self.thrust )

            ## input
            commander.send_setpoint(
                command[0],         ## roll
                command[1],         ## pitch
                command[2],         ## yawRate
                thrust              ## thrust
            )

            sleep(dt)


    ## command should be given in RPY, T
    def send_setpoint_RPY(self, command, n=5):
        ## crazyflie
        cf = self.cf
        ## commander
        commander = cf.commander
        ## timestep
        dt = self.dt / n
        ## current state
        acc_cur = cf.acc

        for _ in range(n):

            ## closed loop
            self.thrust += _dot_thrust(command, acc_cur)      ## integration

            ## cliping
            thrust = _thrust_clip(self.thrust)

            ## input
            commander.send_setpoint(
                command[0],         ## roll
                command[1],         ## pitch
                command[2],         ## yawRate
                thrust              ## thrust
            )

            sleep(dt)


    def stop_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## stop
        self.ready_for_command = False
        commander.send_stop_setpoint()