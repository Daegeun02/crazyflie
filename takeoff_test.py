from numpy import array

from controller import Commander

Kp = 0.707
Kd = 0.300



def takeoff(pos, vel, alt_cmd=1, g=9.81, tol=1e-2):
    """
    current state
    pos: pointer
    vel: pointer

    command
    alt_cmd: desire altitude
    """
    ## command
    pos_cmd = array(0,0,alt_cmd)
    P_pos = 1
    while (P_pos < tol):
        ## difference
        P_pos = pos_cmd - pos    
        D_pos = vel
        ## command
        u = 0
        u += P_pos * Kp
        u += D_pos * Kd
        u += [0,0,g]

        