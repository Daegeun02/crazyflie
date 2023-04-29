## Threading
from threading import Thread


## define Hz of loop
from time import sleep


## for copy memory
from numpy import array, zeros


## control loop
from .integral_loop import _dot_thrust
from .integral_loop import _thrust_clip, alpha


## transformer
from .optimus_prime import _command_as_ENU, _command_as_RPY
from .optimus_prime import _command_is_not_in_there



zero = zeros( 3 )


class Commander(Thread):
    
    
    def __init__(self, scf, dt, control_type='ENU'):
        ## for threading
        super().__init__()
        self.daemon = True

        ## crazyflie
        self.cf = scf.cf
        ## time step
        self.dt = dt

        self.thrust = array([alpha * 9.81], dtype=int)
        ## memory that restores thrust command
        self.ready_for_command = False
        ## command coord
        self.control_type = control_type
        ## store commands
        self.command = zeros(4)
    

    def run(self):
        ## initialize
        cf      = self.cf
        ## target function -> send setpoint
        target  = None
        ## control type
        if self.control_type == 'ENU':
            ## control function
            target = self._send_setpoint_ENU
        elif self.command_type == 'RPY':
            ## control function
            target = self._send_setpoint_RPY
            ## initialize command
            cf.command = array([0,0,0,0])
        else:
            raise ValueError('not supported control type')

        print('ready for guidance start')
    
        ## wait until start flag on
        while not self.ready_for_command:
            sleep(0.1)

        print('guidance is on, start to send command')

        ## start flag is on
        while self.ready_for_command:
            ## read command
            command = array( cf.command )
            ## call control function
            target( command )

        print('mission finished')

    
    def init_send_setpoint(self):
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
        thrust    = self.thrust
        ## timestep
        dt = self.dt / n
        ## acceleration current
        euler_cur = cf.euler_pos
        acc_cur   = cf.acc

        ## transform command
        acc_cmd = _command_is_not_in_there( euler_cur, acc_cmd )
        _command_as_RPY( acc_cmd, command )

        if ( acc_cmd == zero ):
            sleep( dt )
            return 

        for _ in range(n):

            ## closed loop
            thrust[0] += _dot_thrust( command, acc_cur )
            
            ## cliping
            thrust[0] = _thrust_clip( thrust[0] )

            ## input
            commander.send_setpoint(
                command[0],         ## roll
                command[1],         ## pitch
                command[2],         ## yawRate
                thrust[0]           ## thrust
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


    def stop_send_setpoint(self):
        ## commander
        commander = self.cf.commander
        ## stop command
        self.cf.command[:] = zeros(3)
        ## stop signal
        self.ready_for_command = False
        commander.send_stop_setpoint()