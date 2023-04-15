import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.swarm import Swarm

from sensor import start
from sensor import QtmWrapper

from time import sleep



uris             = ['radio://0/80/2M/E7E7E7E714', 'radio://0/80/2M/E7E7E7E705']
rigid_body_names = [ 'cf1', 'cf2' ]
qtm_wrappers     = []

scfs = {
    "uris": uris,
    "rgbn": rigid_body_names
}

if __name__ == "__main__":

    cflib.crtp.init_drivers()

    with Swarm( uris ) as _swarm:

        for rgbn in scfs["rgbn"]:
            qtm_wrappers.append( QtmWrapper( body_name=rgbn ) )
        
        i = 0 

        for uri, scf in _swarm._cfs.items():
            start( scf, qtm_wrappers[i] )
            i += 1

        sleep( 1 )

        t  = 0 
        dt = 0.1

        while t < 20:

            for uri, scf in _swarm._cfs.items():
                cf = scf.cf
                print( cf.posvel, cf.acc )
            print('=' * 20)

            t += dt

            sleep( dt )

        for qtm_wrapper in qtm_wrappers:
            qtm_wrapper.close()

        exit()