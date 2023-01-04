from numpy        import array
from numpy.linalg import norm

from controller import Commander

from visualizer import visualize_flight

Kp = 0.707
Kd = 0.300


        
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

    ## record
    pos_rec = []
    acc_rec = []

    ## loop
    ## takeoff
    print("takeoff")
    P_pos = [0,0,1] - pos
    D_pos = vel
    while ( norm( P_pos ) > tol ):
        print(P_pos)
        # PD loop
        acc_cmd = 0
        acc_cmd += P_pos * Kp
        acc_cmd -= D_pos * Kd
        acc_cmd += [0,0,g]

        ## command
        commander.send_setpoint_ENU(acc_cmd, n)

        ## record
        pos_rec.append(pos)
        acc_rec.append(acc)

        P_pos = [0,0,1] - pos
        D_pos = vel
        
        T.append(T[-1]+dt)
        
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

    # ## hover
    # print("hovering for 1s")
    # t = 0
    # while t < duration:

    #     acc_cmd = [0,0,g]

    #     ## record
    #     pos_rec.append(pos)
    #     acc_rec.append(acc)

    #     commander.send_setpoint_ENU(acc_cmd, n)

    #     t += dt

    #     T.append(T[-1]+dt)

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

    pos_des = []
    acc_des = []

    for p in pos_rec:
        for _ in range(n):
            pos_des.append(p)
    pos_des = array(pos_des)

    for a in acc_rec:
        for _ in range(n):
            acc_des.append(a)
    acc_des = array(acc_des)

    pos_cur = array(commander.pos_rec)
    acc_cur = array(commander.acc_rec)

    T = array(T)
    print(T.shape)
    print(pos_des.shape)
    print(pos_cur.shape)
    print(acc_des.shape)
    print(acc_cur.shape)

    visualize_flight(T, pos_cur, pos_des, acc_cur, acc_des)