## Threading
from threading import Thread


## define Hz of loop
from time import sleep, time


## memory space
from numpy import array, zeros
from numpy.linalg import norm


## control loop
from .acc_att_controller import _dot_thrust
from .acc_att_controller import _thrust_clip, alpha


## transformer
from .optimus_prime import _command_as_ENU, _command_as_RPY
from .optimus_prime import _command_is_not_in_there



class Commander(Thread):
    
    
    def __init__(self, scf, dt, command_type='ENU'):
        ## for threading
        super().__init__(args=(scf,))

        ## crazyflie
        self.cf = scf.cf
        ## time step
        self.dt = dt
        ## memory that restores thrust command
        self.thrust = alpha * 9.81
        ## flag to start read command
        self.ready_for_command = False
        ## command coord
        self.command_type = command_type
    

    def run(self):
        ## initialize
        scf,    = self._args
        target  = None
        rdfcmd  = self.ready_for_command
        flying  = False
        command = [0,0,0]
        ## 
        if self.command_type == 'ENU':
            target = self._send_setpoint_ENU
        elif self.command_type == 'RPY':
            target = self._send_setpoint_RPY
        else:
            raise ValueError('not supported command type')

        ## start
        self._init__send_setpoint()

        while rdfcmd:
            
            if flying:
                try:
                    command = scf.mission.pop(0)
                except:
                    command = array([0,0,9.81])

            else:
                try:
                    command = scf.mission.pop(0)
                    flying  = True
                except:
                    continue

            target( command )


    def join(self):
        super().join()

        self._stop_send_setpoint()

    
    def _init__send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## initialize
        commander.send_setpoint(0, 0, 0, 0)
        self.ready_for_command = True

    
    ## commander should be given in ENU
    def _send_setpoint_ENU(self, acc_cmd, n=5):
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
    def _send_setpoint_RPY(self, command, n=5):
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


    def _stop_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## stop
        self.ready_for_command = False
        commander.send_stop_setpoint()