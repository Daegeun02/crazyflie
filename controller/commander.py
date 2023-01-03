## define Hz of loop
from time import sleep


## memory space
from numpy        import zeros
from numpy.linalg import norm


## control loop
from .acc_att_controller import _dot_thrust
from .acc_att_controller import _thrust_clip, alpha


## transformer
from .optimus_prime import _command_as_ENU, _command_as_RPY



class Commander:
    

    def __init__(self, cf, dt):
        self.cf  = cf
        self.dt  = dt

        self.thrust = alpha * 9.81
        self.pos_rec = []
        self.acc_rec = []
        ## store commands
        self.command = zeros(4)         ## RPY,T

    
    def init_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## initialize
        commander.send_setpoint(0, 0, 0, 0)


    ## command should be given in ENU
    def send_setpoint_ENU(self, acc_cmd, n):
        ## crazyflie
        cf = self.cf
        ## commander
        commander = cf.commander
        command   = self.command
        ## timestep
        dt = self.dt / n
        ## acceleration current
        pos_cur = cf.pos
        acc_cur = cf.acc

        ## transform command
        _command_as_RPY( acc_cmd, command )

        for _ in range(n):
            self.pos_rec.append(pos_cur)
            self.acc_rec.append(acc_cur)

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
            print(command)

            sleep(dt)


    ## command should be given in RPY, T
    def send_setpoint_RPY(self, command, n):
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
        pos_cur = cf.pos
        acc_cur = cf.acc

        ## controller input
        r_cmd, p_cmd, y_cmd, acc_cmd = command

        for _ in range(n):
            self.pos_rec.append(pos_cur)
            self.acc_rec.append(acc_cur)

            ## closed loop
            self.thrust += _dot_thrust(command, acc_cur)      ## integration

            ## cliping
            thrust = _thrust_clip(self.thrust)

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