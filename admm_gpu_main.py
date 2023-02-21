import cupy  as cp
import numpy as np
import numpy.linalg as lg

import class_gpu

from controller import takeoff, hover

import time

def guidance_gpu_2(scf, cf, commander):
    x_0_rt = cf.posvel

    mempool = cp.get_default_memory_pool()
    dt = 0.25
    n = 40  #0~1000
    # x_0 = np.array([0,0,0,0,0,0])
    # x_des =np.array([1.5,0,1.5,0,0,0]) 
    x_des = np.array([0,0,0,0,0,0.2])
    u_ub = 10
    n_rt = n
    optimizer_gpu = class_gpu.ADMM_optimizer_gpu(np.array(x_0_rt), x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
    acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1
    del optimizer_gpu
    mempool.free_all_blocks()

    commander.init_send_setpoint()

    takeoff(scf, commander)

    hover(scf, commander, T=3)

    dt = 0.25
    n = 40  #0~1000
    # x_0 = np.array([0,0,0,0,0,0])
    # x_des =np.array([1.5,0,1.5,0,0,0]) 
    u_ub = 10
    n_rt = n

    print('start')

    for i in range(n):

        mempool = cp.get_default_memory_pool()
        optimizer_gpu = class_gpu.ADMM_optimizer_gpu(np.array(x_0_rt), x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
        acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1

        print(acc_cmd)

        if i > n-4:
            print('break 1')
            break
        if x_0_rt[2] <= 0.22:
            print('break 2')
            break

        commander.send_setpoint_ENU( acc_cmd )

        # time.sleep( dt )

        n_rt = n_rt - 1
        del optimizer_gpu
        mempool.free_all_blocks() 

    command = [0, 0, 0, 0]

    thrust = 45000

    for i in range(20):
        thrust = thrust - 1000
        if thrust > 60000:
            thrust = 60000
        if thrust < 10001:
            thrust = 10001
            
        command[3] = thrust

        commander.send_setpoint_RPY(command)

        time.sleep(0.1)
        if x_0_rt[2] <= 0.1:
            break

    del optimizer_gpu
    mempool.free_all_blocks()

    commander.stop_send_setpoint()