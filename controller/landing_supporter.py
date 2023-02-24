## landing supporter

from numpy import zeros, arange

from time import sleep


class LandingSupporter:

    pass


def landing_supporter(cf, commander, dt=0.1):

    print('landing supporter on')

    command = zeros(3)

    command[2] = 9.81

    step = 0.2

    for _ in range(20):
        command[2] -= step

        cf.command[:] = command 

        sleep(dt)
    
    print('landing')

    commander.stop_send_setpoint()