# Sensor

## 1. IMU

__imu.py__ is a class to use imu in crazyflie drone.<br>

It reads drone's position and velocity in global coordinate and acceleration in body coordiante, basically.<br>
But you can add or request more data from crazyflie. Here's steps for add new data to get from crazyflie.<br>

### way to get data from crazyflie
It basically has two steps. First, make Config function. Second, make callback function. That's it.<br>

In first step, you add config to you crazyflie drone. It's like requesting what you want. use __add_variable__ function as i did, it gets two parameters, Log and DataType. Check below link. This is about what you can request to crazyflie.<br>
Link : [crazyflie logging](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/)
Second, callabck function. It's easy just see what i did. And do as i did.<br>

## 2. Qualisys - Beacon

__qualysis_estimator.py__ is a class to use qualisys's data from beacon.<br>

It reads drone's position, orientation and so on.<br>

Reading data from qualisys looks difficult but it isn't. Qualisys developers has already built structure for us. That is QtmWrapper. You don't need to change any code in QtmWrapper. But if you want to get another data from qualisys then, you just change ___on_packet__ function and __send_pose__ which is callback function of ___on_packet__. Basically, other functions in QtmWrapper is for communication with beacon. Where you read data is in ___on_packet__ function. But you need to focus on __get_parameters__ and __stream_frames__ in ___connect__.<br>

### _on_packet
___on_packet__ function has three steps. First, get data as packet from beacon. Second, unpack and preprocess data as you want. Third, call callback function.<br>
First line of function
```python
header, bodies = packet.get_6d_euler()
```
is where you get data as packet from beacon. 'header' contains communication does work well or something not about drone. And 'bodies' contains what we want. Basically it tracks multiply object at once.
```python
if self.body_name not in self.qtm_6DoF_labels:
    print('Body ' + self.body_name + ' not found.')
``` 
This part talks about you can track multiple object at once.<br>
In that line, there is our second step, unpacking and preprocessing.<br>

Finally, we check whether callback function __on_pose__ is ready, and call callback function. You need to define callback function. About defining callback is in __init.py__.<br>

That's all. 

### stream_frames
__stream_frames__ is about which data you'll read by qualisys.
By changing component, you can get different data from beacon. You can get more detail in definition.