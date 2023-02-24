from controller import Commander

from recorder   import Recorder



def fly(scf):

    commander = Commander(scf, dt=0.1)      ## define commander object
    commander.daemon = True                 ## it must stop when guidance is done

    recorder = Recorder(scf, commander)     ## record flight data
    recorder.daemon = True                  ## it must stop when guidance is done

    commander.start()                       ## thread start
    recorder.start()                        ## thread start