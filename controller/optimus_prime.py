from numpy        import arctan2, arccos, rad2deg
from numpy        import cos, sin, deg2rad
from numpy.linalg import norm



def _thrust_to_RPY( acc_cmd, command ):
    """
    this function translate acc_cmd in NED coordinate,
    to drone's body coordinate roll, pitch, yaw, acc

    command:
    1. roll
    2. pitch
    3. yaw
    4. acc
    """
    ## unpack acceleration command
    aN, aE, aD = acc_cmd    ## NED coodinate

    ## acceleration to z-direction in drone's coordinate
    acc_str = norm( acc_cmd )   ## acceleration strength
    command[3] = acc_str

    ## calculate roll, pitch, yaw
    ## 1. roll
    command[0] = 0      ## this is contraint, [deg]
    ## 2. pitch
    cp = (-1) * aD / acc_str
    pitch_in_rad = arccos( cp )             ## [rad]
    command[1] = rad2deg( pitch_in_rad )    ## [deg]
    ## 3. yaw
    yaw_in_rad = arctan2( aE, aN )          ## [rad]
    command[2] = rad2deg( yaw_in_rad )      ## [deg]


def _thrust_to_NED( command ):
    """
    this function translate command about drone's body coordinate,
    to NED coordinate

    command:
    1. roll
    2. pitch
    3. yaw
    4. acc
    """
    ## unpack command
    roll, pitch, yaw, acc_str = command     ## [deg]

    ## deg to rad
    roll  = deg2rad( roll )                   ## [rad]
    pitch = deg2rad( pitch )                  ## [rad]
    yaw   = deg2rad( yaw )                    ## [rad]

    ## basic
    cp, sp = cos( pitch ), sin( pitch )
    cy, sy = cos( yaw )  , sin( yaw )

    ## calculate NED command
    aN = (-1) * acc_str * sp * cy
    aE = (-1) * acc_str * sp * sy
    aD = (-1) * acc_str * cp

    ## return 
    return aN, aE, aD
