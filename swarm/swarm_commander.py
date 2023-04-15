from threading import Thread

from cflib.crazyflie.swarm import Swarm

from controller import Commander

from sensor import start
from sensor import QtmWrapper

from time import sleep



class SwarmCommander(Swarm, Thread):


    def __init__(self, uris, dt):

        super().__init__( uris )

        self.dt = dt

        self.thread_is_alive = False

        self.qtm_wrappers = []

    
    def start_sensor(self, rgbn):

        for uri, scf in self._cfs.items():

            qtm_wrapper = QtmWrapper( body_name=rgbn[uri] )

            self.qtm_wrappers.append( qtm_wrapper )

            start( scf, qtm_wrapper )

        sleep( 1 )


    def run(self):

        threads = []
        reporter = self.Reporter()

        self.thread_is_alive = True

        ## spawn commander
        for uri, scf in self._cfs.items():
            
            thread = Commander( scf, self.dt )
            threads.append( thread )
            thread.start()
        
        ## go go
        for thread in threads:
            thread.init_send_setpoint()

        ## monitoring
        while self.thread_is_alive:

            if reporter.is_error_reported():
                error = reporter.errors[0]
                raise Exception( ' Error ' ) from error

        ## end.. Let's go to home
        for thread in threads:
            thread.stop_send_setpoint()


    def stop_all(self):

        self.thread_is_alive = False

        for qtm_wrapper in self.qtm_wrappers:
            qtm_wrapper.close()