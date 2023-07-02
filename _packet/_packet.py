from serial import Serial

from numpy import zeros
from numpy import float32, uint8
from numpy import ndarray



class _Packet( Serial ):

    def _enroll( self, size: int, header: ndarray ):

        self.TxData = zeros( size, dtype=float32 )
        self.header = ( header.astype( uint8 ) ).tobytes()


    def _sendto( self ):

        buffer = self.header + self.TxData.tobytes()

        self.write( buffer )