import cflib.crtp

from cflib.crazyflie.swarm import Swarm

from swarm import SwarmCommander

from sensor import start
from sensor import QtmWrapper

from time import sleep



uris = [
    'radio://0/80/2M/E7E7E7E714', 
    'radio://0/80/2M/E7E7E7E705'
]

rigid_body_names = { 
    uris[0]: 'cf1', 
    uris[1]: 'cf2'
}

qtm_wrappers     = {}

scfs = {
    "uris": uris,
    "rgbn": rigid_body_names
}


if __name__ == "__main__":

    cflib.crtp.init_drivers()

    with SwarmCommander( uris ) as _swarm:
        ## turn on logging
        _swarm.start_sensor( rigid_body_names )
        ## turn on commander
        _swarm.start()

        t  = 0 
        dt = 0.1

        while t < 20:

            for uri, scf in _swarm._cfs.items():
                cf = scf.cf
                print( cf.posvel, cf.acc )
            print('=' * 20)

            t += dt

            sleep( dt )
        
        _swarm.stop_all()

        exit()