from threading import Thread

import xml.etree.cElementTree as ET

import qtm
import asyncio

from math import isnan

from numpy import array

from time import time



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
            components=['6deuler', '6d'],
            on_packet=self._on_packet)

    async def _discover(self):
        async for qtm_instance in qtm.Discover('192.168.254.1'):
            return qtm_instance

    def _on_packet(self, packet):
        header, bodies = packet.get_6d()
    
        if self.body_name not in self.qtm_6DoF_labels:
            print('Body ' + self.body_name + ' not found.')
        else:
            ## who i am
            index = self.qtm_6DoF_labels.index(self.body_name)
            ## where i am and which orientation i have
            if bodies is None:
                rot = None
            else:
                temp_cf_data = bodies[index]

                ## body to ENU coordinate
                r = temp_cf_data[1].matrix
                rot = [
                    [r[0], r[3], r[6]],
                    [r[1], r[4], r[7]],
                    [r[2], r[5], r[8]],
                ]

        header, bodies = packet.get_6d_euler()

        if bodies is None:
            return

        if self.body_name not in self.qtm_6DoF_labels:
            print('Body ' + self.body_name + ' not found.')
        else:
            ## who i am
            index = self.qtm_6DoF_labels.index(self.body_name)
            ## where i am and which orientation i have
            temp_cf_data = bodies[index]

            position = temp_cf_data[0]
            x = position[0] / 1000              ## m
            y = position[1] / 1000              ## m
            z = position[2] / 1000              ## m

            euler = temp_cf_data[1]
            R = euler[2]                        ## deg
            P = euler[1]                        ## deg
            Y = euler[0]                        ## deg

            if self.on_pose:
                # Make sure we got a position
                if isnan(x):
                    return

                self.on_pose([x, y, z, R, P, Y, rot])

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()


class SendPose:

    # pre_t = 0
    # pre_t = cls.pre_t
    # cur_t = time()
    # dt    = cur_t - pre_t
    # cls.pre_t = cur_t

    @classmethod
    def transmit_pose( cls, pose ):

        pass


    @classmethod
    def send_extpose(cls, cf, pose):
        if pose[6] is None:
            cls.send_pos(
                cf, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
            )
        else:
            cls.send_pose(
                cf, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]
            )


    @classmethod
    def send_pose(cls, cf, x, y, z, R, P, Y, rot):
        cf.posvel[0] = x 
        cf.posvel[1] = y 
        cf.posvel[2] = z

        cf.extpos.send_extpos(x, y, z)

        cf.euler_pos[0] = R
        cf.euler_pos[1] = P
        cf.euler_pos[2] = Y

        cf.rot = array(rot)
    
    
    @classmethod
    def send_pos(cls, cf, x, y, z, R, P, Y):
        cf.posvel[0] = x 
        cf.posvel[1] = y 
        cf.posvel[2] = z

        cf.extpos.send_extpos(x, y, z)

        cf.euler_pos[0] = R
        cf.euler_pos[1] = P
        cf.euler_pos[2] = Y