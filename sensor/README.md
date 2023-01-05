# Sensor

## 0. Usage
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
That's all. It starts update __cf's__ pose immediatly.<br>

## 1. IMU

__imu.py__ is a class to use imu in crazyflie drone.<br>

It reads drone's position and velocity in global coordinate and acceleration in body coordiante, basically.<br>
But you can add or request more data from crazyflie. Here's steps for add new data to get from crazyflie.<br>

### way to get data from crazyflie
It basically has two steps. First, make Config function. Second, make callback function. That's it.<br>

In first step, you add config to you crazyflie drone. It's like requesting what you want. use __add_variable__ function as i did, it gets two parameters, Log and DataType. Check below link. This is about what you can request to crazyflie.<br>
Link : [crazyflie logging](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/)<br>
Second, callabck function. It's easy just see what i did. And do as i did.<br>

## 2. Qualisys - Beacon

__qualysis_estimator.py__ is a class to use qualisys's data from beacon.<br>

It reads drone's position, orientation and so on.<br>

Reading data from qualisys looks difficult but it isn't. Qualisys developers has already built structure for us. That is QtmWrapper. You don't need to change any code in QtmWrapper. But if you want to get another data from qualisys then, you just change ___on_packet__ function and __send_pose__ which is callback function of ___on_packet__. Basically, other functions in QtmWrapper is for communication with beacon. Where you read data is in ___on_packet__ function. But you need to focus on __get_parameters__ and __stream_frames__ in ___connect__.<br>

### _on_packet
___on_packet__ function has three steps. First, get data as packet from beacon. Second, unpack and preprocess data as you want. Third, call callback function.<br>
First line of function
```python
header, bodies = packet.get_6d()
header, bodies = packet.get_6d_euler()
```
is where you get data as packet from beacon. 'header' contains communication does work well or something not about drone. And 'bodies' contains what we want. Basically it tracks multiply object at once.<br>

From __packet.get_6d__, we read drone's rotation matrix which rotate drone's body coordinate to ENU coordinate. This rotation matrix is for record drone's acc properly in ENU coordinate. And __packet.get_6d_euler__, we read drone's position and euler angle's. This is for drone's guidance and get velocity by numerical differential.<br>

```python
if self.body_name not in self.qtm_6DoF_labels:
    print('Body ' + self.body_name + ' not found.')
``` 
This part talks about you can track multiple object at once.<br>
In that line, there is our second step, unpacking and preprocessing.<br>

Finally, we check whether callback function __on_pose__ is ready, and call callback function. You need to define callback function. About defining callback is in __init.py__.<br>

### SendPose
SendPose class is our callback function. You will define callback function as like
```python
callback_function = lambda pose: SendPose.send_extpose( cf, pose )
```
In send_extpose function, it calls two function optionally. __send_pos__ is for send without rotation matrix from __packet.get_6d__ and __send_pose__ is for send with rotation matrix from __packet.get_6d_euler__. Also, in each functions called by __send_extpose__, measure 'dt'. That 'dt' is pretty accurate time step of position difference.<br>

### stream_frames
__stream_frames__ is about which data you'll read by qualisys.
By changing component, you can get different data from beacon. You can get more detail in definition.

## 3. init.py
__init.py__ is simple. It has setup function each initialize kalman filter in crazyflie imu, and so on. And get memory space in crazyflie object to store read data from sensors.<br>