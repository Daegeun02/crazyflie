# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2019 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a Qualisys QTM system and feed the position to a
Crazyflie. It uses the high level commander to upload a trajectory to fly a
figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
rigid_body_name to match the name of the Crazyflie in QTM.
"""
# Qulaisys
import asyncio
from imp import cache_from_source
import math
import time
from tkinter import N
import xml.etree.cElementTree as ET
from threading import Thread
from matplotlib import markers
import pandas as pd
import cupy as cp

import qtm
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.crazyflie.commander import Commander
from cflib.crtp.crtpstack import CRTPPacket

from qtm import QRTPacket, QRTConnection
from qtm.packet import QRTComponentType




# Guidance
import numpy as np
import math as m
import numpy as np
import numpy.linalg as lg
import matplotlib.pyplot as plt
import scipy.sparse.linalg as sla
# import admm_guidance as guid
# import qualisys_LQR 
import class_gpu 

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E710')

# The name of the rigid body in QTM that represents the Crazyflie
rigid_body_name = 'cf1'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generatedy cf
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

global pos_cb , vel_cb, euler_cb, angvel_cb
global spec_time, t_gui_st, t_gui_end, const_thrust
pos_cb = [0,0,0]
vel_cb = [0,0,0]
euler_cb = [0,0,0]
angvel_cb = [0,0,0]
spec_time = []
t_gui_st = 0
t_gui_end = 0
thrusts_ = []
const_thrust = 47000    #43300

data_posvel = pd.DataFrame(columns=['x','y','z','vx','vy','vz'])
data_real = pd.DataFrame(columns=['roll','pitch','yaw', 'ax', 'ay', 'az'])
data_cmd = pd.DataFrame(columns=['roll_cmd', 'pitch_cmd', 'yaw','ax_cmd', 'ay_cmd', 'az_cmd'])
# data_cmd = pd.DataFrame(columns=['roll_cmd','pitch_cmd','yawRate_cmd','thrust','ax_cmd','ay_cmd','az_cmd'])


class QtmWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while (self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        self.connection = await qtm.connect('127.0.0.1')
        
        if self.connection is None:
            print("Failed to connect")
            return


        params = await self.connection.get_parameters(parameters=['6d'])
        self.params = params
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip() for index, label in enumerate(xml.findall('*/Body/Name'))]

        await self.connection.stream_frames(
            components=['6D'],
            on_packet=self._on_packet)

    async def _discover(self):
        async for qtm_instance in qtm.Discover('192.168.254.1'):
            return qtm_instance

    def _on_packet(self, packet):
        header, bodies = packet.get_6d()

        if bodies is None:
            return

        if self.body_name not in self.qtm_6DoF_labels:
            print('Body ' + self.body_name + ' not found.')
        else:
            index = self.qtm_6DoF_labels.index(self.body_name)
            temp_cf_pos = bodies[index]
            x = temp_cf_pos[0][0] / 1000
            y = temp_cf_pos[0][1] / 1000
            z = temp_cf_pos[0][2] / 1000

            r = temp_cf_pos[1].matrix
            rot = [
                [r[0], r[3], r[6]],
                [r[1], r[4], r[7]],
                [r[2], r[5], r[8]],
            ]

            if self.on_pose:
                # Make sure we got a position
                if math.isnan(x):
                    return

                self.on_pose([x, y, z, rot])

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()


class Uploader:
    def __init__(self):
        self._is_done = False
        self._success = True

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done, write_failed_cb=self._upload_failed)

        while not self._is_done:
            time.sleep(0.2)

        return self._success

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True
        self._success = True

    def _upload_failed(self, mem, addr):
        print('Data upload failed')
        self._is_done = True
        self._success = False


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=10)
    log_config.add_variable('kalman.varPX', 'FP16')
    log_config.add_variable('kalman.varPY', 'FP16')
    log_config.add_variable('kalman.varPZ', 'FP16')

    

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def _sqrt(a):
    """
    There might be rounding errors making 'a' slightly negative.
    Make sure we don't throw an exception.
    """
    if a < 0.0:
        return 0.0
    return math.sqrt(a)


def send_extpose_rot_matrix(cf, x, y, z, rot):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotaton matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """
    quat = Rotation.from_matrix(rot).as_quat()

    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
    else:
        cf.extpos.send_extpos(x, y, z)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)

def position_callback(timestamp, data, logconf):
    pos_cb[0] = data['kalman.stateX']
    pos_cb[1] = data['kalman.stateY']
    pos_cb[2] = data['kalman.stateZ']
    vel_cb[0] = data['stateEstimate.vx']
    vel_cb[1] = data['stateEstimate.vy']
    vel_cb[2] = data['stateEstimate.vz']
    df = pos_cb + vel_cb

    data_posvel.loc[len(data_posvel)] = df
    # data_posvel.append(df, ignore_index=True)
    # data_posvel = pd.concat([data_posvel, df], axis=0)


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('kalman.stateX', 'FP16')
    log_conf.add_variable('kalman.stateY', 'FP16')
    log_conf.add_variable('kalman.stateZ', 'FP16')
    log_conf.add_variable('stateEstimate.vx', 'FP16')
    log_conf.add_variable('stateEstimate.vy', 'FP16')
    log_conf.add_variable('stateEstimate.vz', 'FP16')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def position_callback_euler(timestamp, data, logconf):
    euler_cb[0] = data['stateEstimate.roll']
    euler_cb[1] = data['stateEstimate.pitch']
    euler_cb[2] = data['stateEstimate.yaw']
    ax, ay, az = data['acc.x']*9.81, data['acc.y']*9.81, data['acc.z']*9.81
    spec_time.append(time.time())
    df_real = euler_cb + [ax, ay, az]
    data_real.loc[len(data_real)] = df_real  # 'roll','pitch','yawRate', 'ax', 'ay', 'az'



def start_position_printing_euler(scf):
    log_conf = LogConfig(name='Stabilizer', period_in_ms=10)
    log_conf.add_variable('stateEstimate.roll', 'FP16')
    log_conf.add_variable('stateEstimate.pitch', 'FP16')
    log_conf.add_variable('stateEstimate.yaw', 'FP16')
    log_conf.add_variable('acc.x', 'FP16')
    log_conf.add_variable('acc.y', 'FP16')
    log_conf.add_variable('acc.z', 'FP16')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback_euler)
    log_conf.start()

def stop_logconf(scf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    # log_conf.add_variable('stabilizer.roll', 'float')
    # log_conf.add_variable('stabilizer.pitch', 'float')
    # log_conf.add_variable('stateEstimateZ.rateYaw', 'float')
    # log_conf.add_variable('acc.x', 'float')
    # log_conf.add_variable('acc.y', 'float')
    # log_conf.add_variable('acc.z', 'float')

    # scf.cf.log.add_config(log_conf)
    # log_conf.data_received_cb.add_callback(position_callback_euler)
    log_conf.stop()



def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

def controller_acc(h, h_cmd, h_dot, axis_, zeta, w):
    K_h = w**2  
    K_v = 2*zeta*w
    g = 0
    if axis_ == 'z':
        g = 9.81
    a_h = -K_h*(h-h_cmd) - K_v*h_dot + g
    thrust_ = 10001 + ((const_thrust-10001)/9.81) * a_h   #39300
    return a_h

def test_z(cf, commander, desX, desY, desZ, flight_mode, ):
    # t_gui_st = time.time()
    for i in range(50):
        # thrust_z = controller_acc(pos_cb[2], desZ, vel_cb[2], 'z', zeta=0.9, w=1)
        # thrust_y = controller_acc(pos_cb[1], desY, vel_cb[1], 'y', zeta=0.9, w=1)
        # thrust_x = controller_acc(pos_cb[0], desX, vel_cb[0], 'x', zeta=0.9, w=1)
        # thrust = int(lg.norm([thrust_x, thrust_y, thrust_z]))
        a_z = controller_acc(pos_cb[2], desZ, vel_cb[2], 'z', zeta=0.9, w=1)
        a_y = controller_acc(pos_cb[1], desY, vel_cb[1], 'y', zeta=0.9, w=1)
        a_x = controller_acc(pos_cb[0], desX, vel_cb[0], 'x', zeta=0.9, w=1)
        thrust = int(10001 + ((const_thrust-10001)/9.81) * lg.norm([a_x, a_y, a_z]))

        roll_ =  -math.degrees(np.arctan2(a_y, a_z))
        pitch_ = math.degrees(np.arctan2(a_x, a_z))
        yawRate_ = 0.02*math.degrees(euler_cb[2])
       
        if thrust > 60000:
            thrust = 60000
        if thrust < 10001:
            thrust = 10001
        commander.send_setpoint(roll_, pitch_,yawRate_, thrust)
        time.sleep(0.1)
        if flight_mode == 'down':
            if pos_cb[2] <=0.3:
                print("----------",i)
                # commander.send_stop_setpoint()
                break
        # else:
        #     if i > 10:  
        #         if abs(vel_cb[2]) < 0.1:
        #             return 
        
    t_gui_end = time.time()
    
    if flight_mode == 'dowm':
        for i in range(100):
            if  thrust <= 10001:
                thrust = 10001
            commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(0.1)
            thrust = thrust - 400
            # thrust = thrust - 150
            if pos_cb[2] <= 0.09 or thrust == 10001:
                commander.send_stop_setpoint()
                return
def sleep_while_checking_stable(scf: SyncCrazyflie, tf_sec, dt_sec=0.1, check_low_battery=True):
    if tf_sec == 0:
        return
    log_config = LogConfig(name='Roll', period_in_ms=int(dt_sec*1000.0))
    log_config.add_variable('stabilizer.roll', 'float')
    log_config.add_variable('pm.vbat', 'float')
    t_sec = 0
    batt_lowpass = 3.7
    RC = 0.25 # time constant in seconds of first order low pass filter
    alpha = dt_sec/(RC + dt_sec)
    # print('sleeping {:10.4f} seconds'.format(tf_sec))
    with SyncLogger(scf, log_config) as sync_logger:
        for log_entry in sync_logger:
            log_data = log_entry[1]
            roll = log_data['stabilizer.roll']
            batt = log_data['pm.vbat']
            # if np.abs(roll) > 60:
            #     raise UnstableException("flip detected {:10.4f} deg, for {:s}".format(roll, scf.cf.link_uri))
            # batt_lowpass = (1 - alpha)*batt_lowpass + alpha*batt
            # #print("battery {:10.4f} V, for {:s}".format(batt_lowpass, scf.cf.link_uri))
            # if batt_lowpass < 2.9 and check_low_battery:
            #     raise LowBatteryException("low battery {:10.4f} V, for {:s}".format(batt, scf.cf.link_uri))
            # t_sec += dt_sec
            # if t_sec>tf_sec:
            #     return
def guidance_gpu(cf):
    commander = cf.commander
    dt = 0.5
    n = 20  #0~1000
    x_des = np.array([0,0,0,0,0,0])
    # x_0 =np.array([1.5,0,1.5,0,0,0]) 
    u_ub = 100
    n_rt = n
    
    commander.send_hover_setpoint(0,0,0,1.5)
    time.sleep(1.5)
    for i in range(5):
        commander.send_hover_setpoint(0,0,0,1.5)
        if i != 4:
            time.sleep(0.5)
    
    commander.send_setpoint(0,0,0,0)
    x_0_rt = np.array(pos_cb + vel_cb)

    for i in range(n - 2):
        # print('1')
        t1 = time.time()
        mempool = cp.get_default_memory_pool()
        optimizer_gpu = class_gpu.ADMM_optimizer_gpu(x_0_rt, x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
        acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1
        # print('acc_cmd:', acc_cmd)
        ax, ay, az = acc_cmd 
        roll_ =  -math.degrees(np.arctan2(ay, az))
        pitch_ = math.degrees(np.arctan2(ax, az))
        yawRate_ = 0.02*math.degrees(euler_cb[2])
        thrust = int(10001 + ((const_thrust-10001)/9.81)*lg.norm([ax,ay,az]))
        if thrust > 60000:
            thrust = 60000
        if thrust < 10001:
            thrust = 10001
        t2 = time.time()
        commander.send_setpoint(roll_, pitch_,yawRate_, thrust)
        # time.sleep(dt)
        data_cmd.loc[len(data_cmd)] = [roll_, pitch_, yawRate_, ax, ay, az]
        
        if pos_cb[2] < 0.01:
            commander.send_stop_setpoint()
            break
        x_0_rt = np.array(pos_cb + vel_cb)
        n_rt = n_rt - 1
        # print(lg.norm([ax,ay,az]))
        del optimizer_gpu
        mempool.free_all_blocks()
    commander.send_stop_setpoint()
    time.sleep(dt)

def guidance_cmd(acc_cmd):
    ax, ay, az = acc_cmd 
    roll_ =  -math.degrees(np.arctan2(ay, az))
    pitch_ = math.degrees(np.arctan2(ax, az))
    yawRate_ = 0.02*math.degrees(euler_cb[2])
    thrust = int(10001 + ((const_thrust-10001)/9.81)*lg.norm([ax,ay,az]))
    if thrust > 60000:
        thrust = 60000
    if thrust < 10001:
        thrust = 10001
    return roll_, pitch_, yawRate_, thrust


def guidance_gpu_2(cf):
    commander = cf.commander
    mempool = cp.get_default_memory_pool()
    x_0_rt = np.array(pos_cb + vel_cb)
    dt = 0.25
    n = 40  #0~1000
    # x_0 = np.array([0,0,0,0,0,0])
    # x_des =np.array([1.5,0,1.5,0,0,0]) 
    x_des = np.array([0,0,0,0,0,0.2])
    u_ub = 10
    n_rt = n
    optimizer_gpu = class_gpu.ADMM_optimizer_gpu(x_0_rt, x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
    acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1
    del optimizer_gpu
    mempool.free_all_blocks()
    commander.send_hover_setpoint(0, 0, 0, 1.5)
    time.sleep(2)
    dt = 0.25
    n = 40  #0~1000
    # x_0 = np.array([0,0,0,0,0,0])
    # x_des =np.array([1.5,0,1.5,0,0,0]) 
    u_ub = 10
    n_rt = n
    commander.send_setpoint(0,0,0,0)
    x_0_rt = np.array(pos_cb + vel_cb)
    for i in range(n):
        # print('1')
        t1 = time.time()
        mempool = cp.get_default_memory_pool()
        optimizer_gpu = class_gpu.ADMM_optimizer_gpu(x_0_rt, x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
        acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1
        roll_, pitch_, yawRate_, thrust = guidance_cmd(acc_cmd)
        t2 = time.time()
        if i > n-4:
            break
        # if lg.norm(acc_cmd) > u_ub + 2:
        #     # print(lg.norm(acc_cmd))
        #     # time.sleep(0.1)
        #     break
        if pos_cb[2] <= 0.22:
            break
        commander.send_setpoint(roll_, pitch_,yawRate_, thrust)
        time.sleep(dt )
        data_cmd.loc[len(data_cmd)] = [roll_, pitch_, yawRate_] + list(np.squeeze(acc_cmd, 1))
        print(lg.norm(acc_cmd))
        x_0_rt = np.array(pos_cb + vel_cb)
        n_rt = n_rt - 1
        del optimizer_gpu
        mempool.free_all_blocks() 
    # for i in range(3):
    #     roll_, pitch_, yawRate_, thrust = guidance_cmd(optimizer_gpu.x[3*i: 3*(i+1)])
    #     print(lg.norm(acc_cmd))
    #     if i !=2:
    #         time.sleep(dt)

    for i in range(20):
        thrust = thrust - 1000
        if thrust > 60000:
            thrust = 60000
        if thrust < 10001:
            thrust = 10001
        commander.send_setpoint(0, 0,0, thrust)
        print(thrust)
        time.sleep(0.1)
        if pos_cb[2] <= 0.1:
            break
    # for i in range(100):
    #     roll_, pitch_, yawRate_, thrust = guidance_cmd([0, 0,vel_cb[2]**2/(2*pos_cb[2])+9.81])
    #     commander.send_setpoint(0, 0,0, thrust)
    #     time.sleep(0.1)
    #     print(1)
    #     if pos_cb[2] <= 0.1:
    #         break
    del optimizer_gpu
    mempool.free_all_blocks()


    # for i in range(4):
    #     commander.send_hover_setpoint(0, 0, 0, pos_cb[2])
    #     time.sleep(0.2)
    # commander.send_hover_setpoint(0, 0, 0, pos_cb[2])

    # x_des =np.array([1.5,0,0,0,0,vel_cb[2]])
    # x_0_rt = np.array(pos_cb + vel_cb)
    # n = 20
    # n_rt = n
    # dt = 0.25

    # for i in range(n):
    #     # t1 = time.time()
    #     mempool = cp.get_default_memory_pool()
    #     optimizer_gpu = class_gpu.ADMM_optimizer_gpu(x_0_rt, x_des, n_rt*dt, u_ub, n_rt, 0.01, 0.01, 0.01, 0.01, 1500, 3*n_rt)
    #     acc_cmd = optimizer_gpu.ADMM_opt_gpu(30, 30)  # shape of acc_cmd ; 3X1
    #     roll_, pitch_, yawRate_, thrust = guidance_cmd(acc_cmd)
    #     # t2 = time.time()
    #     print(t2 - t1)
    #     if i > n-4:
    #         break
    #     # if lg.norm(acc_cmd) > u_ub + 2:
    #     #     # print(lg.norm(acc_cmd))
    #     #     # time.sleep(0.1)
    #     #     break
    #     commander.send_setpoint(roll_, pitch_,yawRate_, thrust)
    #     time.sleep(dt)
    #     data_cmd.loc[len(data_cmd)] = [roll_, pitch_, yawRate_] + list(np.squeeze(acc_cmd, 1))
    #     print(lg.norm(acc_cmd))
    #     x_0_rt = np.array(pos_cb + vel_cb)
    #     n_rt = n_rt - 1
    #     del optimizer_gpu
    #     mempool.free_all_blocks() 
    # for i in range(3):
    #     roll_, pitch_, yawRate_, thrust = guidance_cmd(optimizer_gpu.x[3*(i): 3*(i+1)])
    #     time.sleep(dt)
    # del optimizer_gpu
    # mempool.free_all_blocks()


        

    # cf = scf.cf  # type: Crazyflie
    # commander = cf.high_level_commander  # type: cflib.HighLevelCOmmander
    # for i in range(8):
    #     commander.send_hover_setpoint(0, 0, 0, pos_cb[2] - 0.1)
    #     time.sleep(0.5)
    # commander.send_hover_setpoint(0, 0, 0, 0.2)
    # time.sleep(0.5)
    # time.sleep(5)
    # sleep_while_checking_stable(scf, tf_sec=5, check_low_battery=False)

    # time.sleep(0.1)
    # hi_commander = cf.high_level_commander
    # hi_commander.land(0.2, 5)
    # hi_commander.stop()
    # commander.send_hover_setpoint(0,0,0, 0.2)
    # time.sleep(2)
    commander.send_stop_setpoint()
    # commander.send_hover_setpoint(0,0,0,1.5)
    # time.sleep(1.5)
    # for i in range(5):
    #     commander.send_hover_setpoint(0,0,0,1.5)
    #     if i != 4:
    #         time.sleep(0.5)

def test_LQR(cf):

    commander = cf.commander
    # commander.send_hover_setpoint(0,0,0,1.5)
    # time.sleep(2)
    # commander.send_hover_setpoint(0,0,0,1.5)
    # time.sleep(0.5)

    # test_z(cf, commander, 1,1,1.5, flight_mode='up')
    dt = 0.1  # --> time step : 0~ 99
    N = 100  
    # x_0 = [1,1,1.5,0,0,0]

    commander.send_hover_setpoint(0,0,0,1.3)
    time.sleep(2)
    # for i in range(30):
    #     commander.send_hover_setpoint(0,0,0,1.3)
    #     time.sleep(0.1)
    x_0 = pos_cb + vel_cb
    # lqr_obj = qualisys_LQR.lqr_controller(x_0, N)

    # for i in range(2): 
    #     commander.send_hover_setpoint(0,0,0,1.5)
    #     time.sleep(1.5)
    # commander.send_hover_setpoint(0,0,0,1.5)
    # time.sleep(2)
    # for i in range(100):
    #     commander.send_hover_setpoint(0,0,0,1.5)
    #     time.sleep(0.1)
    commander.send_setpoint(0, 0, 0, 0)

    for i in range(N):
        dt = 0.1
        # t_go = lg.norm(pos_cb)/(lg.norm(vel_cb))   # time to go
        
        # time_step = int(t_go/dt)    # step to go 
        time_step = N-1 - i
        x = pos_cb + vel_cb
        x = np.array(x)
        x[2] = x[2] - 0.2
        idx_step = (N - 1) - time_step   # ex) 100 --> 0
        u = lqr_obj.K[:,6*idx_step:6*idx_step+6]@x[:,np.newaxis] + lqr_obj.L[:,idx_step:idx_step+1]
        print(i, pos_cb, u)
        roll_ =  -math.degrees(np.arctan2(u[1], u[2]))
        pitch_ = math.degrees(np.arctan2(u[0],  u[2]))
        yawRate_ = 0.02*math.degrees(euler_cb[2])
        thrust = int(10001 + ((const_thrust-10001)/9.81) * lg.norm(u))
        # thrust = 60000 if thrust > 60000 else ()
        if thrust > 60000:
            thrust = 60000
        if thrust < 10001:
            thrust = 10001
        commander.send_setpoint(roll_, pitch_,yawRate_, thrust)
        time.sleep(0.1)
        if pos_cb[2] <= 0.2:    #0.05
            print("--------------------",i)
            # commander.send_stop_setpoint()
            break
    for i in range(100):
        thrust = thrust - 750
        if thrust < 10001:
            thrust = 10001
        commander.send_setpoint(0,0,0, thrust)
        time.sleep(0.1)
        if pos_cb[2] <= 0.03 or thrust==10001:    #0.05
            commander.send_stop_setpoint()
            break

    commander.send_stop_setpoint()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect to QTM
    qtm_wrapper = QtmWrapper(rigid_body_name)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1

        # Set up a callback to handle data from QTM
        qtm_wrapper.on_pose = lambda pose: send_extpose_rot_matrix(
            cf, pose[0], pose[1], pose[2], pose[3])

        # time.sleep(1)

        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        activate_high_level_commander(cf)
        # activate_mellinger_controller(cf)
        reset_estimator(cf)
        # run_sequence(cf, trajectory_id, duration)
        start_position_printing(scf)
        start_position_printing_euler(scf)
        # commander = cf.commander
        # commander.send_setpoint(0, 0, 0, 0)
        # test_z(cf, commander,  pos_cb[0], pos_cb[1], 1.8, flight_mode='up')
        # test_z(cf, commander,  0, 0, 0, flight_mode='down')
        guidance_gpu_2(cf)
        # stop_logconf(cf)
        # test_LQR(cf)
        



    qtm_wrapper.close()



    ### plots
    print(data_posvel)
    fig = plt.figure(1)
    axes = fig.subplots(6,3)
    for idx, i in enumerate(data_posvel.columns):
        axes[idx,0].plot(data_posvel[i], label=i)
        axes[idx,0].set_title(i)
   
    for idx, i in enumerate(data_real.columns):
        axes[idx,1].plot(data_real[i], label=i)
        axes[idx,1].set_title(i)
    for idx, i in enumerate(data_cmd.columns):
        axes[idx,1].plot(data_cmd[i], label=i)
        axes[idx,1].set_title(i)

    plt.show()