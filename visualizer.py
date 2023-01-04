import matplotlib.pyplot as plt
from numpy import array



def visualize_flight(T, pos_cur, pos_des, acc_cur, acc_des):
    plt.figure(figsize=(11,7), dpi=200)

    plt.subplot(321)
    plt.plot(T, pos_cur[:,0], label='position x')
    plt.plot(T, pos_des[:,0], label='desination x')
    plt.legend()
    plt.grid()

    plt.subplot(322)
    plt.plot(T, pos_cur[:,1], label='position y')
    plt.plot(T, pos_des[:,1], label='desination y')
    plt.legend()
    plt.grid()

    plt.subplot(323)
    plt.plot(T, pos_cur[:,2], label='position z')
    plt.plot(T, pos_des[:,2], label='desination z')
    plt.legend()
    plt.grid()

    plt.subplot(324)
    plt.plot(T, acc_cur[:,0], label='position x')
    plt.plot(T, acc_des[:,0], label='desination x')
    plt.legend()
    plt.grid()

    plt.subplot(325)
    plt.plot(T, acc_cur[:,1], label='position y')
    plt.plot(T, acc_des[:,1], label='desination y')
    plt.legend()
    plt.grid()

    plt.subplot(326)
    plt.plot(T, acc_cur[:,2], label='position z')
    plt.plot(T, acc_des[:,2], label='desination z')
    plt.legend()
    plt.grid()

    plt.show()


def visualize_acc_norm(acc_rec, acc_ref, t):
    plt.figure(figsize=(6,6), dpi=200)

    plt.plot(t, acc_rec, label='record')
    plt.plot(t, acc_ref, label='referenc')
    plt.legend()
    plt.grid()

    plt.show()


def visualize_acc(acc_rec, acc_ref, t):
    plt.figure(figsize=(6,6), dpi=200)

    plt.subplot(311)
    plt.plot(t, acc_rec[:,0], label='x1')
    plt.plot(t, acc_ref[:,0], label="reference")
    plt.legend()
    plt.grid()

    plt.subplot(312)
    plt.plot(t, acc_rec[:,1], label='y1')
    plt.plot(t, acc_ref[:,1], label="reference")
    plt.legend()
    plt.grid()

    plt.subplot(313)
    plt.plot(t, acc_rec[:,2], label='z1')
    plt.plot(t, acc_ref[:,2], label="reference")
    plt.legend()
    plt.grid()

    plt.show()

def visualize_thrust(timestep, record, referen, thrust, alpha):
    plt.figure(figsize=(11,5), dpi=200)

    plt.subplot(121)
    plt.plot(timestep, record, label="actual acc")
    plt.plot(timestep, referen, label="reference acc")
    plt.legend()
    plt.grid()

    plt.subplot(122)
    plt.plot(timestep, thrust, label="actual thrust")
    plt.plot(timestep, array(referen) * alpha, label="reference thrust")
    plt.legend()
    plt.grid()

    plt.show()

def visualize_state(pos, vel, acc, t):
    plt.figure(figsize=(9,9), dpi=200)

    plt.subplot(331)
    plt.plot(t, pos[0,:], label="position X")
    plt.legend()
    plt.grid()

    plt.subplot(332)
    plt.plot(t, pos[1,:], label="position Y")
    plt.legend()
    plt.grid()

    plt.subplot(333)
    plt.plot(t, pos[2,:], label="position Z")
    plt.legend()
    plt.grid()

    plt.subplot(334)
    plt.plot(t, vel[0,:], label="velocity X")
    plt.legend()
    plt.grid()

    plt.subplot(335)
    plt.plot(t, vel[1,:], label="velocity Y")
    plt.legend()
    plt.grid()

    plt.subplot(336)
    plt.plot(t, vel[2,:], label="velocity Z")
    plt.legend()
    plt.grid()

    plt.subplot(337)
    plt.plot(t, acc[0,:], label="acceleration X")
    plt.legend()
    plt.grid()

    plt.subplot(338)
    plt.plot(t, acc[1,:], label="acceleration Y")
    plt.legend()
    plt.grid()

    plt.subplot(339)
    plt.plot(t, acc[2,:], label="acceleration Z")
    plt.legend()
    plt.grid()

    plt.show()