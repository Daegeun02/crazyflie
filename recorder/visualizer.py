import matplotlib.pyplot as plt



def plot_acc_pos_cmd(acc, acccmd, vel, velest, pos, posest, _len):

    plt.figure(figsize=(14,14))

    plt.subplot(331)
    plt.plot(acc[0,:_len], label='realtime acceleration x')
    plt.plot(acccmd[0,:_len], label='command input x')
    plt.legend()
    plt.grid()

    plt.subplot(332)
    plt.plot(acc[1,:_len], label='realtime acceleration y')
    plt.plot(acccmd[1,:_len], label='command input y')
    plt.legend()
    plt.grid()

    plt.subplot(333)
    plt.plot(acc[2,:_len], label='realtime acceleration z')
    plt.plot(acccmd[2,:_len], label='command input z')
    plt.legend()
    plt.grid()

    ## velocity is reversed
    plt.subplot(334)
    plt.plot(vel[0,:_len], label='realtime velocity x')
    plt.plot(velest[0,:_len], label='estimated velocity x')
    plt.legend()
    plt.grid()

    plt.subplot(335)
    plt.plot(vel[1,:_len], label='realtime velocity y')
    plt.plot(velest[1,:_len], label='estimated velocity y')
    plt.legend()
    plt.grid()

    plt.subplot(336)
    plt.plot(vel[2,:_len], label='realtime velocity z')
    plt.plot(velest[2,:_len], label='estimated velocity z')
    plt.legend()
    plt.grid()

    plt.subplot(337)
    plt.plot(pos[0,:_len], label='realtime position x')
    plt.plot(posest[0,:_len], label='estimated position x')
    plt.legend()
    plt.grid()

    plt.subplot(338)
    plt.plot(pos[1,:_len], label='realtime position y')
    plt.plot(posest[1,:_len], label='estimated position y')
    plt.legend()
    plt.grid()

    plt.subplot(339)
    plt.plot(pos[2,:_len], label='realtime position z')
    plt.plot(posest[2,:_len], label='estimated position z')
    plt.legend()
    plt.grid()

    plt.show()


def plot_thrust(thrust, thrustcmd, _len):

    plt.figure(figsize=(8,8))

    plt.plot(thrust[:_len], label='realtime thrust')
    plt.plot(thrustcmd[:_len], label='reference thrust')

    plt.legend()
    plt.grid()

    plt.show()


def plot_att(att, attcmd, _len):

    plt.figure(figsize=(14,14))
    
    plt.subplot(311)
    plt.plot(att[0,:_len], label='realtime euler x')
    plt.plot(attcmd[0,:_len], label='realtime euler input x')
    plt.legend()
    plt.grid()

    plt.subplot(312)
    plt.plot(att[1,:_len], label='realtime euler y')
    plt.plot(attcmd[1,:_len], label='realtime euler input y')
    plt.legend()
    plt.grid()

    plt.subplot(313)
    plt.plot(att[2,:_len], label='realtime euler z')
    plt.plot(attcmd[2,:_len], label='realtime euler input z')
    plt.legend()
    plt.grid()

    plt.show()