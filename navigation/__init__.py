from threading import Thread

from .imu       import IMU
from .qualisys  import Qualisys

from _packet import _Packet

from time import sleep



class Navigation( Thread ):

    def __init__( self, config ):

        super().__init__( self )

        self.packet = None
        self.header = config['header']
        self.cf     = config['scf'].cf

        self.imu = IMU( config['scf'] )
        self.qtm = Qualisys( config['body_name'] )

        self.AllGreen = True

        self._on_link( config['port'], config['baud'] )


    def _on_link( self, port, baud ):

        self.packet = _Packet( port, baud, timeout=1 )


    @staticmethod
    def _on_pose( cf, data: list ):
        
        cf.pos[:] = data[0:3]
        cf.rpy[:] = data[3:6]

    
    def run( self ):

        pos = self.cf.pos
        vel = self.cf.vel
        acc = self.cf.acc
        rpy = self.cf.rpy

        self.imu.start_get_vel()
        self.imu.start_get_acc()

        self.qtm.on_pose = lambda data: self._on_pose( self.cf, data )

        if self.packet is not None:
            packet = self.packet

            packet._enroll( 4*12, self.header )

        while self.AllGreen:

            packet.TxData[0:3] = pos
            packet.TxData[3:6] = vel
            packet.TxData[6:9] = acc
            packet.TxData[9: ] = rpy

            packet._sendto()

            sleep( 0.01 )