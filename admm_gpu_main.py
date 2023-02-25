import cupy  as cp
import numpy as np
import numpy.linalg as lg

import class_gpu

from controller import takeoff, hover, goto
from controller import landing_supporter

import time

from numpy import array

def guidance_gpu_2(scf, cf, commander):

    print('optimal guidance start')
    x_0_rt = cf.posvel

    mempool = cp.get_default_memory_pool()
    dt = 0.25
    n = 40  #0~1000
    # x_0 = np.array([0,0,0,0,0,0])
    # x_des =np.array([1.5,0,1.5,0,0,0]) 
    x_des = np.array([0,0,0,0,0,0])
    u_ub = 10
    n_rt = n
    optimizer_gpu = class_gpu.ADMM_optimizer_gpu(np.array(x_0_rt), x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
    acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1
    del optimizer_gpu
    mempool.free_all_blocks()

    takeoff(scf, commander)

    hover(scf, commander, T=3)

    goto(scf, array([1,1,1.5]), commander)

    hover(scf, commander, T=2)

    # dt = 0.25
    # n = 40  #0~1000
    dt = 0.1
    n = 100
    # x_0 = np.array([0,0,0,0,0,0])
    # x_des =np.array([1.5,0,1.5,0,0,0]) 
    u_ub = 10
    n_rt = n

    print('start')

    cf.destination[:] = x_des[:3]

    for i in range(n):

        mempool = cp.get_default_memory_pool()
        optimizer_gpu = class_gpu.ADMM_optimizer_gpu(np.array(x_0_rt), x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
        acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1

        cf.command[:] = acc_cmd.reshape(3,)

        if i > n-4:
            print('break 1')
            break
        if x_0_rt[2] <= 0.1:
            print('break 2')
            break

        time.sleep( dt )

        n_rt = n_rt - 1
        del optimizer_gpu
        mempool.free_all_blocks() 

    landing_supporter(cf, commander)

    commander.stop_send_setpoint()

    del optimizer_gpu
    mempool.free_all_blocks()