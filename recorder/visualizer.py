import matplotlib.pyplot as plt



def plot_acc_pos_cmd(acc, pos, cmd, _len):

    plt.figure(figsize=(14,7))

    plt.subplot(231)
    plt.plot(acc[0,:_len], label='realtime acceleration x')
    plt.plot(cmd[0,:_len], label='command input x')
    plt.legend()
    plt.grid()

    plt.subplot(232)
    plt.plot(acc[1,:_len], label='realtime acceleration y')
    plt.plot(cmd[1,:_len], label='command input y')
    plt.legend()
    plt.grid()

    plt.subplot(233)
    plt.plot(acc[2,:_len], label='realtime acceleration z')
    plt.plot(cmd[2,:_len], label='command input z')
    plt.legend()
    plt.grid()

    plt.subplot(234)
    plt.plot(pos[0,:_len], label='realtime position x')
    plt.legend()
    plt.grid()

    plt.subplot(235)
    plt.plot(pos[1,:_len], label='realtime position y')
    plt.legend()
    plt.grid()

    plt.subplot(236)
    plt.plot(pos[2,:_len], label='realtime position z')
    plt.legend()
    plt.grid()

    plt.show()


def plot_thrust(thrust, _len):

    plt.figure(figsize=(8,8))

    plt.plot(thrust[0,:_len], label='realtime thrust')

    plt.legend()
    plt.grid()

    plt.show()


def plot_vel(vel, _len):

    plt.figure(figsize=(8,8))

    plt.plot(vel[0,:_len], label='realtime velocity x')
    plt.plot(vel[1,:_len], label='realtime velocity y')
    plt.plot(vel[2,:_len], label='realtime velocity z')

    plt.legend()
    plt.grid()

    plt.show()


def plot_att(att, attimu, attcmd, _len):
    
    plt.subplot(311)
    plt.plot(att[0,:_len], label='realtime euler x')
    plt.plot(attimu[0,:_len], label='realtime euler from imu x')
    plt.plot(attcmd[0,:_len], label='realtime euler input x')
    plt.legend()
    plt.grid()

    plt.subplot(312)
    plt.plot(att[1,:_len], label='realtime euler y')
    plt.plot(attimu[1,:_len], label='realtime euler from imu y')
    plt.plot(attcmd[1,:_len], label='realtime euler input y')
    plt.legend()
    plt.grid()

    plt.subplot(313)
    plt.plot(att[2,:_len], label='realtime euler z')
    plt.plot(attimu[2,:_len], label='realtime euler from imu z')
    plt.plot(attcmd[2,:_len], label='realtime euler input z')
    plt.legend()
    plt.grid()

    plt.show()