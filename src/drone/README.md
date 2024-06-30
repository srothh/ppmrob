# Drone

Drone node handles the communication with the drone. ROS Action Servers listen to commands from other nodes and relay them to the drone. Status information from the drone is published using ROS-Publishers.

Tello drone is supported using a custom developed driver: `tello.py`. It supports sending multiple commands to the drone and receiving answers from the drone. Also the telemetry data from the drone is collected and sent to registered handlers which in turn publish relevant data to ROS Topics. Image data is also collected for publishing.
 
## Actions

| action  | argument  |   |
|---|---|---|
| launch  | bool takeoff  | true: takeoff <br> false: land  |
| move |  geometry_msgs/Transform target  | movement in x,y,z and rotation in z |
|  command | string command | execute sdk command  | 

## Sensors

sensordata published in topics

| publisher  | topic | msg type |comment |
|---|---|---|
| TwistSensor  | /drone/twist |  geometry_msgs/TwistStamped | velocity in x,y,z. with timestamp |
| BatterySensor  | /drone/battery | std_msgs/UInt8 | battery level in % |
| ImageSensor  | /drone/camera | Image | Image data | 

### BatterySensor 

pubishes the status of the battery read from the tello status string
 
 ```
 $ rostopic echo /drone/battery
---
data: 66
---
data: 66
---
 ```


### TwistSensor

reads fields from the status string broadcasted by the drone

```
$ rostopic echo /drone/twist

header: 
  seq: 3076
  stamp: 
    secs: 1708002179
    nsecs: 882850885
  frame_id: "velocity"
twist: 
  linear: 
    x: 10.0
    y: 0.0
    z: 0.0
  angular: 
    x: 0.0
    y: 0.0
    z: 0.0
```

### available fields from the tello status string

- pitch:%d
- roll:%d
- yaw:%d
- vgx:%d
- vgy%d
- vgz:%d
- templ:%d
- temph:%d
- tof:%d
- h:%d
- bat:%d
- baro:%.2f
- time:%d
- agx:%.2f
- agy:%.2f
- agz:%.2f

### ImageSensor

publishes image data from the camera.

* links:
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython



## Notes

* if battery < 70 strange things happen, 
** error No valid imu
** error no joystick
