# crazyflie
acc feedback controller for crazyflie and etc...

This repository is source code for crazyflie wrapped library...<br>
Only for ACSL...

There's three big package to use crazyflie drone...<br>
In each package folder, there's README file to explain details about library...<br>

## 1. sensor
This package is to make easy to use crazyflie imu sensor and qualisys's beacon<br>
You just type this code to use sensor data...<br>
### Usage
```python 
from sensor import start
from sensor import QtmWrapper

rigid_body_name = 'as you define in qualisys'

qtm_wrapper = QtmWrapper(body_name=rigid_body_name)

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

    cf = scf.cf

    start(scf, qtm_wrapper)

qtm_wrapper.close()
```
<br>
That's all. It starts update __cf's__ pose immediatly.<br>

## 2. controller
This package is to make easy to use crazyflie control function __send_setpoint__.

### Usage
```python
from controller import Commander

## commander
commander = Commander(cf, dt)
commander.init_send_setpoint()

## state
pos = cf.pos
vel = cf.vel

## flight
acc_cmd = guidance_function( pos, vel )

commander.send_setpoint_ENU( acc_cmd )

## end flight
commander.stop_send_setpoint()
```

## 3. filter

![test](./image/test_image.png)
