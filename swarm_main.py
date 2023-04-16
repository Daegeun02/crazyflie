import cflib.crtp

from swarm import *

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

        init_cfs( _swarm._cfs )

        takeoff( _swarm._cfs )

        landing( _swarm._cfs )

        _swarm.stop_all()

        exit()