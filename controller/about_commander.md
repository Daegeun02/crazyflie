# Commander

__commander.py__ is a class to control crazyflie drone<br>

This class wraps crazyflie's send_setpoint function.<br>
There's one feedback loop to make sure that drone follow the command.<br>
Basically feedback loop get 10 Hz in 1 Hz of commander's send_setpoint function.<br>

It has three functions.

## init_send_setpoint
This function initialize crazyflie's send_setpoint function.<br>
Just insert $[0,\ 0,\ 0,\ 0]$ as command.

## send_setpoint
You can use this function as crazyflie's send_setpoint function.<br>
The main difference between this one and crazyflie's one is the input of function.
This function get __acc_cmd__ in __NED__ like $[aN,\ aE,\ aD]$, 
but crazyflie's get __command__ like $[roll,\ pitch,\ yaw,\ thrust]$.<br>
Because we thought it is more convenient for user who use our send_setpoint function.<br>
Thanks for __optimus_prime.py__ we can translate $[aN,\ aE,\ aD]$ to $[roll,\ pitch,\ yaw,\ thrust]$.

You can choose Hz of the feedback loop $1 \approx 10$ Hz and Hz of this function is limited in 10 Hz.<br>

Here's about feedback loop in function
> To Be Continue..

## stop_setpoint
It's simple.<br>
Just stop sending setpoint.<br>
We use crazyflie's __send_stop_setpoint__ function.<br>

