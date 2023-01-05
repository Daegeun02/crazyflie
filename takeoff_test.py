from numpy        import array, linspace
from numpy.linalg import norm

from controller import Commander

from visualizer import visualize_flight, visualize_acc, visualize_acc_norm

Kp = array([1.404, 1.404, 0.707])
Kd = array([1.000, 1.000, 1.000])



def takeoff(cf, destination=[0,0,1], g=9.81, tol=1e-1):
    ## timestep
    T = linspace(0,5,51)
    dt = T[1] - T[0]
    
    ## commander
    commander = Commander(cf, dt)
    commander.init_send_setpoint()

    ## state
    pos   = cf.pos
    vel   = cf.vel

    ## loop
    ## takeoff
    print("takeoff")
    destination = pos + array([0,0,1])
    P_pos = destination - pos
    D_pos = vel
    for _ in T:
        # PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,g]

        ## command
        commander.send_setpoint_ENU(acc_cmd)

        P_pos = destination - pos
        D_pos = vel

    print("moving")
    destination = array([1,1,1.5])
    P_pos = destination - pos
    D_pos = vel
    for _ in T:
        # PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,g]

        ## command
        commander.send_setpoint_ENU(acc_cmd)

        P_pos = destination - pos
        D_pos = vel

    commander.stop_send_setpoint()

    # ## record
    acc_rec = array(commander.acc_rec)
    acc_cmd = array(commander.acc_cmd)
    acc_rec_norm = array(commander.acc_rec_norm)
    acc_cmd_norm = array(commander.acc_cmd_norm)
    # eul_rec = array(commander.eul_rec)
    # eul_cmd = array(commander.eul_cmd)

    _len = len(acc_rec)

    t = linspace(0,5,_len)

    # visualize_acc(eul_rec, eul_cmd, t)
    # visualize_acc(acc_rec, acc_cmd, t)
    visualize_acc(acc_rec, acc_cmd, t)
    visualize_acc_norm(acc_rec_norm, acc_cmd_norm, t)
        

def takeoff_and_land(cf, destination=[1,1,1], duration=1, landing=[2,2,0], g=9.81, tol=1e-1):
    ## timestep
    dt = 0.1
    T  = [0]
    
    ## Hz of PD loop
    n = 5

    ## commander
    commander = Commander(cf, dt)
    commander.init_send_setpoint()

    ## state
    pos   = cf.pos
    vel   = cf.vel
    acc   = cf.acc

    ## loop
    ## takeoff
    print("takeoff")
    P_pos = [0,0,1] - pos
    D_pos = vel
    while ( ( norm( P_pos ) > tol ) or ( norm( D_pos ) > tol) ):
        # PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,g]
        print(acc_cmd)

        ## command
        commander.send_setpoint_ENU(acc_cmd, n)

        P_pos = [0,0,1] - pos
        D_pos = vel

        print('=' * 20)
        
    # print("moving")
    # P_pos = destination - pos
    # D_pos = vel
    # while ( norm( P_pos ) > tol ):
    #     print(P_pos)
    #     ## PD loop
    #     acc_cmd = 0
    #     acc_cmd += P_pos * Kp
    #     acc_cmd -= D_pos * Kd
    #     acc_cmd += [0,0,g]

    #     ## command
    #     commander.send_setpoint_ENU(acc_cmd, n)

    #     ## record
    #     pos_rec.append(pos)
    #     acc_rec.append(acc)

    #     P_pos = destination - pos
    #     D_pos = vel
        
    #     T.append(T[-1]+dt)

    ## hover
    print("hovering for 1s")
    t = 0
    while t < duration:

        acc_cmd = [0,0,g]

        commander.send_setpoint_ENU(acc_cmd, n)

        t += dt

    # ## land
    # print("landing")
    # P_pos = landing - pos
    # D_pos = vel
    # while ( norm( P_pos ) > tol):
    #     print(P_pos)
    #     acc_cmd = 0
    #     acc_cmd += P_pos * Kp
    #     acc_cmd -= D_pos * Kd
    #     acc_cmd += [0,0,g]

    #     ## record
    #     pos_rec.append(pos)
    #     acc_rec.append(acc)

    #     commander.send_setpoint_ENU(acc_cmd, n)

    #     P_pos = landing - pos
    #     D_pos = vel

    #     T.append(T[-1]+dt)

    commander.stop_send_setpoint()