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

## 3. filter

![test](./image/test_image.png)
