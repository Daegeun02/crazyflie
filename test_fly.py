import time 
from numpy.linalg import norm
from numpy import zeros, ones, ones_like, linspace

from controller import Commander

from filter import LPF

from visualizer import visualize_thrust



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
    mission = ones_like(timestep)
    referen = ones(len(timestep)*n)
    t = linspace(0,timestep[-1],len(timestep)*n)
    ## timestep
    mission1 = mission[:21] * 10.00
    mission4 = mission[21:] * 9.81
    missions = []
    missions.append(mission1)
    missions.append(mission4)

    ## initialize coef
    dt = timestep[1] - timestep[0]

    ## commander
    commander = Commander(cf, 111, dt)

    commander.init_send_setpoint()

    idx = 0

    for i in range(len(missions)):
        _mission = missions[i]
        acc_cmd = [0, 0, _mission[0]]

        for _ in range(len(_mission)):

            roll_, pitch_, yawRate_, acc = guidance_cmd(acc_cmd, dt)

            referen[idx:idx+n] *= acc

            commander.send_setpoint([roll_, pitch_, yawRate_, acc], n=n)

            idx += n

    commander.stop_setpoint()

    record1 = commander.record1
    record2 = commander.record2

    visualize_thrust(t, record1, referen, record2)