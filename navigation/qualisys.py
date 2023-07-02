from threading import Thread

from serial import Serial

import xml.etree.cElementTree as ET

import qtm
import asyncio

from math import isnan



class Qualisys( Thread ):

    def __init__( self, body_name ):

        Thread.__init__( self )

        self.body_name  = body_name
        self.on_pose    = None
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True

        self.start()


    def close( self ):

        self._stay_opne = False

        self.join()


    def run( self ):

        asyncio.run( self._life_cycle() )


    async def _life_cycle( self ):

        await self._connect()

        while ( self._stay_open ):

            await asyncio.sleep( 1 )

        await self._close()


    async def _connect( self ):

        self.connection = await qtm.connect( '127.0.0.1' )

        if self.connection is None:
            print( "Failed to connect" )
            return

        params      = await self.connection.get_parameters( parameters=['6d'] )
        self.params = params

        xml = ET.fromstring( params )

        self.qtm_6DoF_labels = [label.text.strip() for index, label in enumerate(xml.findall( '*/Body/Name' ) ) ]

        await self.connection.stream_frames(
            components=[ '6deuler' ],
            on_packet =self._on_packet
        )

    
    async def _close( self ):

        await self.connection.stream_frames_stop()
        self.connection.disconnect()


    async def _discover( self ):

        async for qtm_instance in qtm.Discover( '192.168.254.1' ):
            return qtm_instance


    def _on_packet( self, packet ):

        header, bodies  = packet.get_6d_euler()

        if self.body_name not in self.qtm_6DoF_labels:
            print( 'Body' + self.body_name + 'not found.' )
        else:
            index = self.qtm_6DoF_labels.index( self.body_name )

            if bodies is None:
                pass
        
            else:
                data = bodies[index]

                position = data[0]
                x = position[0] / 1000
                y = position[1] / 1000
                z = position[2] / 1000

                euler = data[1]
                R = euler[2]
                P = euler[1]
                Y = euler[0]

                if self.on_pose:
                    if isnan( x ):
                        return

                    self.on_pose( [x, y, z, R, P, Y] )