## controller package


## commander
"""
this class is what you should use
"""
from .commander_v_01 import Commander
# from .commander_v_01 import Commander as Commander_v_01

from .hl_commander_v_01 import takeoff, landing, hover, goto
# from .hl_commander_v_01 import takeoff as takeoff_v_01
# from .hl_commander_v_01 import landing as landing_v_01
# from .hl_commander_v_01 import hover   as hover_v_01


## optimus prime
"""
translator -> transform -> transformation -> transformer
"""
from .optimus_prime import _command_as_ENU, _command_as_RPY


## acc_att_controller
from .acc_att_controller import _dot_thrust 
from .acc_att_controller import _thrust_clip
from .acc_att_controller import alpha, Kp