from numpy.linalg import norm
from numpy import zeros_like, ones, linspace, append

from controller import Commander
from controller import alpha

from visualizer import visualize_thrust

## takeoff
from takeoff_test import takeoff



def guidance_cmd(acc_cmd, dt):
    ## roll, pitch, yaw
    roll_    = 0
    pitch_   = 0
    yawRate_ = 0
    ## acceleration
    acc = norm(acc_cmd)
    ## return 
    return roll_, pitch_, yawRate_, acc


def test_fly(timestep, cf, n=10):
    ## mission
    t = linspace(0,timestep[-1],len(timestep)*n)
    referen = zeros_like(t)
    ## timestep
    mission = ones(9) * 10.5
    mission = append(mission, linspace(10.5,9,11))
    mission = append(mission, ones(11)*9)
    mission = append(mission, ones(20)*9.8)

    if len(mission) != len(timestep):
        raise ValueError("you are idiot")

    ## initialize coef
    dt = timestep[1] - timestep[0]

    ## commander
    commander = Commander(cf, dt)

    commander.init_send_setpoint()

    idx = 0

    for cmd in mission:
        acc_cmd = [0,0,cmd]

        roll_, pitch_, yawRate_, acc = guidance_cmd(acc_cmd, dt)

        referen[idx:idx+n] = acc

        commander.send_setpoint_RPY(cmd=[roll_, pitch_, yawRate_, acc], n=n)

        idx += n

    commander.stop_send_setpoint()

    record1 = commander.record1
    record2 = commander.record2

    visualize_thrust(t, record1, referen, record2, alpha)