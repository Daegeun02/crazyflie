import matplotlib.pyplot as plt



def plot_acc_pos_cmd(acc, pos, cmd, _len):

    plt.figure(figsize=(10,5))

    plt.subplot(211)

    plt.plot(acc[:_len], label='realtime acceleration')
    plt.plot(cmd[:_len], label='command input')

    plt.legend()
    plt.grid()

    plt.subplot(212)

    plt.plot(pos[:_len], label='realtime position')

    plt.legend()
    plt.grid()

    plt.show()


def plot_thrust(thrust, _len):

    plt.figure(figsize=(8,8))

    plt.plot(thrust[:_len], label='realtime thrust')

    plt.legend()
    plt.grid()

    plt.show()


def plot_vel(vel, _len):

    plt.figure(figsize=(8,8))

    plt.plot(vel[:_len], label='realtime thrust')

    plt.legend()
    plt.grid()

    plt.show()