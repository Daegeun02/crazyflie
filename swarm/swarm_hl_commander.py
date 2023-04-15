from threading import Thread

from controller.integral_loop import smooth_command

from numpy import zeros

from numpy.linalg import norm

from time import sleep



from constants import read_constant

_const = read_constant('gain') 
Kp     = _const["Kp"]
Kd     = _const["Kd"]

_const = read_constant('gravity')
g      = _const["g"]

careg    = zeros( 3 )
careg[2] += g


curs    = {}
dess    = {}
descmds = {}
acccmds = {}


def init_cfs( cfs ):

    for uri, scf in cfs.items():
        curs[uri] = zeros( 6 )
        dess[uri] = zeros( 6 )

        descmds[uri] = zeros( 6 )
        acccmds[uri] = zeros( 6 )


def controller( acccmd, descmd, posvel ):

    acccmd[:] = 0
    acccmd[:] += ( descmd[:3] - posvel[:3] ) * Kp
    acccmd[:] += ( descmd[3:] - posvel[3:] ) * Kd
    acccmd[:] += careg


def takeoff( cfs, h=1.5, T=3, dt=0.1 ):

    N = int( T / dt )

    def _func( *args ):
        uri = args[0]
        scf = args[1]
        N   = args[2]

        _cf = scf.cf

        t = 0

        cur    = curs[uri]
        des    = dess[uri]
        descmd = descmds[uri]
        acccmd = acccmds[uri]

        cur[:] = _cf.posvel
        posvel = _cf.posvel

        ## takeoff straight up
        des[ 0 ] = cur[0]
        des[ 1 ] = cur[1]
        des[ 2 ] = h
        des[3: ] = 0

        _cf.destination[:] = des

        for _ in range( N ):
            ## one way to delete overshoot
            descmd[:] = smooth_command( des, cur, t, int(T/2) )

            controller( acccmd, descmd, posvel )

            _cf.command[:] = acccmd

            t += dt

            sleep( dt )


    func = _func()

    threads = []
    
    for uri, scf in cfs.items():
        args = [ uri, scf, N ]

        thread = Thread(target=func, args=args)
        threads.append( thread )
        thread.start()

    for thread in threads:
        thread.join()


def landing( cfs, h=0.2, T=3, dt=0.1 ):

    N = int( T / dt )

    ## landing supporter
    step = 0.075

    def _func( *args ):
        uri = args[0]
        scf = args[1]
        N   = args[2]

        _cf = scf.cf

        t = 0

        cur    = curs[uri]
        des    = dess[uri]
        descmd = descmds[uri]
        acccmd = acccmds[uri]

        cur[:] = _cf.posvel
        posvel = _cf.posvel

        ## landing straight down
        des[ 0 ] = cur[0]
        des[ 1 ] = cur[1]
        des[ 2 ] = h
        des[3: ] = 0

        _cf.destination[:] = des

        for k in range( N ):
            ## one way to delete overshoot
            descmd[:] = smooth_command( des, cur, t, T )

            controller( acccmd, descmd, posvel )

            _cf.command[:] = acccmd

            if norm( posvel - des ) < 0.1:
                break

            t += dt

            sleep( dt )

        ## landing supporter
        for _ in range( N - k ):

            careg[2] -= step

            controller( acccmd, descmd, posvel )

            _cf.command[:] = acccmd
        
        careg[2] = g

    func = _func()

    threads = []

    for uri, scf in cfs.items():
        args = [ uri, scf, N ]

        thread = Thread( target=func, args=args )
        threads.append( thread )
        thread.start()

    for thread in threads:
        thread.join()