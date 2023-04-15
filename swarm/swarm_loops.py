from numpy import array



def _acc_command_loop(scf, commander):

    cf = scf.cf

    target = commander.run

    while commander.ready_for_command:
        ## read command
        command = array( cf.command )
        ## call control function
        target( command )


def _vel_command_loop(scf, commander):
    pass