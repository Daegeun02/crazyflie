from cflib.crazyflie.swarm import Swarm

from ..controller import Commander

from .swarm_loops import _acc_command_loop
from .swarm_loops import _vel_command_loop



def _swarm_commander(scfs: dict, dt=0.1):
    commanders = {}

    for uri, scf in scfs.items():
        commanders[uri] = Commander(scf.cf, dt)

    return commanders


class SwarmCommander(Swarm):

    def spmc_by_acc_start(self):
        """
        spmc: Single Processor Multi Command 
        """
        self.parallel_safe(
            _acc_command_loop, 
            args_dict=_swarm_commander(self._cfs)
        )

    
    def spmc_by_vel_start(self):
        """
        spmc: Single Processor Multi Command 
        """
        self.parallel_safe(
            _vel_command_loop,
            args_dict=_swarm_commander(self._cfs)
        )

    
    def upload_acc_cmd(self, acccmds: dict):

        for uri, scf in self._cfs.items():
            scf.cf.command[:] = acccmds[uri]
