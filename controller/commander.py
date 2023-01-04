## define Hz of loop
from time import sleep


## memory space
from numpy import zeros, array


## control loop
from .acc_att_controller import _dot_thrust
from .acc_att_controller import _thrust_clip, alpha


## transformer
from .optimus_prime import _command_as_ENU, _command_as_RPY
from .optimus_prime import _command_is_not_in_there



class Commander:
    

    def __init__(self, cf, dt):
        self.cf  = cf
        self.dt  = dt

        self.thrust = alpha * 9.81
        self.acc_rec = []
        self.acc_cmd = []
        self.eul_rec = []
        self.eul_cmd = []
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
        euler_cur = cf.euler_pos

        ## transform command
        acc_cmd = _command_is_not_in_there( euler_cur, acc_cmd )
        _command_as_RPY( acc_cmd, command )

        for _ in range(n):
            acc_cur = cf.rot @ cf.acc
            # eul_cur = array(euler_cur)
            # eul_cmd = array(command[:3])

            self.acc_rec.append(acc_cur)
            self.acc_cmd.append(acc_cmd)

            # self.eul_rec.append(eul_cur)
            # self.eul_cmd.append(eul_cmd)

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
    def send_setpoint_RPY(self, command, n):
        ## crazyflie
        cf = self.cf
        ## commander
        commander = cf.commander
        ## timestep
        dt = self.dt / n
        ## current state
        pos_cur = cf.pos
        acc_cur = cf.acc

        ## controller input
        r_cmd, p_cmd, y_cmd, acc_cmd = command

        for _ in range(n):
            self.acc_rec.append(acc_cur)

            ## closed loop
            self.thrust += _dot_thrust(command, acc_cur)      ## integration

            ## cliping
            thrust = _thrust_clip(self.thrust)

            ## input
            commander.send_setpoint(r_cmd, p_cmd, y_cmd, thrust)

            sleep(dt)


    def stop_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## stop
        commander.send_stop_setpoint()