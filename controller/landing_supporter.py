## landing supporter

from numpy import zeros, array

from time import sleep

Kp = array([4.000,4.000,2.000])
Kd = array([2.700,2.700,2.100])



class LandingSupporter:

    pass


def landing_supporter(cf, commander, dt=0.1, step=0.075):

    print('landing supporter on')

    posvel = cf.posvel

    des = array([0,0,posvel[2]])

    command = zeros(3)

    g = 9.82

    for _ in range(20):
        P_pos = des - posvel[:3]
        D_pos = posvel[3:]

        command[:] = 0
        command[:] += P_pos * Kp
        command[:] -= D_pos * Kd
        command[:] += [0,0,9.81]

        g -= step

        command[2] = g

        cf.command[:] = command 

        sleep(dt)
    
    print('landing')

    commander.stop_send_setpoint()