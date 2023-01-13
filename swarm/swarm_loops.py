from numpy import array

from time  import sleep



def _acc_command_loop(scf, commander):
    is_flying = False           ## flag

    while commander.ready_for_command:

        if is_flying:
            try:
                command = scf.mission.pop(0)
            except:
                command = array([ 0, 0,9.8])        ## for hovering
    
        else:
            try:
                command = scf.mission.pop(0)
                is_flying = True
            except:
                sleep(0.1)
                continue
        
        commander.send_setpoint_ENU( command )


def _vel_command_loop(scf, commander):
    pass