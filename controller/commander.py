## define Hz of loop
from time import sleep


## memory space
from numpy import zeros


## control loop
from .acc_att_controller import _dot_thrust , _dot_acceleration
from .acc_att_controller import _thrust_clip, alpha


## transformer
from .optimus_prime import _command_as_ENU, _command_as_RPY



class Commander:
    

    def __init__(self, cf, dt):
        self.cf  = cf
        self.dt  = dt

        self.thrust = alpha * 9.81
        self.record1 = []
        self.record2 = []
        ## store commands
        self.command = zeros(4)         ## RPY,T
        ## acceleration compansation
        self.acc_com = zeros(3)         ## ENU

    
    def init_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## initialize
        commander.send_setpoint(0, 0, 0, 0)

    
    ## command should be given in ENU
    def send_setpoint_ENU(self, cmd, n):
        ## crazyflie
        cf = self.cf
        ## commander
        commander = cf.commander
        command   = self.command
        acc_com   = self.acc_com
        ## timestep
        dt = self.dt / n
        ## acceleration current
        acc_cur = cf.acc

        for _ in range(n):

            ## closed loop
            acc_com += _dot_acceleration( cmd, acc_cur )      ## integration

            ## command in ENU
            acc_cmd = cmd + acc_com
        
            ## transform command
            _command_as_RPY( acc_cmd, command )

            ## input
            commander.send_setpoint(
                command[0],         ## roll
                command[1],         ## pitch
                command[2],         ## yawRate
                command[3]          ## thrust
            )

            sleep(dt)


    ## command should be given in RPY, T
    def send_setpoint_RPY(self, cmd, n):
        ## crazyflie
        cf = self.cf
        ## commander
        commander = cf.commander
        ## to debug
        record1 = self.record1
        record2 = self.record2
        ## timestep
        dt = self.dt / n
        ## current state
        acc_cur = cf.acc

        ## controller input
        r_cmd, p_cmd, y_cmd, acc_cmd = cmd

        for _ in range(n):

            ## closed loop
            self.thrust += _dot_thrust(acc_cmd, acc_cur)      ## integration

            ## cliping
            thrust, self.thrust = _thrust_clip(self.thrust)

            ## input
            commander.send_setpoint(r_cmd, p_cmd, y_cmd, thrust)

            ## record
            record1.append(acc_cur[2]) 
            record2.append(thrust)

            sleep(dt)


    def stop_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## stop
        commander.send_stop_setpoint()