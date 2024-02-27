# ppmrob
## Initial docker setup
Currently listener + publisher example. To run use

``
docker compose up --build
``

To exit run

``
docker compose down
``
## Custom nodes
To create custom nodes, recreate the folder structure for your node, edit the variables PACKAGE_NAME (name of the package/node you want to create) and LAUNCH_FILE(name of the launch file in your directory for this node) in the Dockerfile
and add the node/service to the docker-compose.yml file.

## Drone

### test

#### tello cli
to test sending commands and receiving responses

```bash
/catkin_ws/src/drone/src# python3 tello_ctrl_obj.py
```

test protokol:

1 poweron drone

2 connect to wifi

3 start cli client

4 'command' -> watch for receiving 'ok' and drone starts blinking green

5 'takeoff'

6 'land' 


#### tello status
test receiving tello status messages on port 8890

```bash
/catkin_ws/src/drone/src# python3 tello_status.py
```

startup and test node

```bash
#configure ros environment

$ source devel/setup.bash

#start emergency safety node

$ rosrun drone safety_node.py

#start action servers

$ rosrun drone drone_node.py

# start image listener client

$ rosrun drone image_listener.py

$ rosrun tello cv_node.py
# start movement action client

$ rosrun drone action_client.py

```

#### Troubleshooting

* error No valid imu

* error no joystick

** battery < 70

** low light 

* unstable flight

** low light

### Actions

| action  | argument  |   |
|---|---|---|
| launch  | bool order  | true: takeoff <br> false: land  |
| move |  geometry_msgs/Transform  | movement in x,y,z and rotation in z |
|  command | string  | execute sdk command  | 

### Sensors

sensordata published in topics

| publisher  | topic | msg type |comment |
|---|---|---|
| TwistSensor  | /drone/twist |  geometry_msgs/TwistStamped | velocity in x,y,z. with timestamp |
| BatterySensor  | /drone/battery | std_msgs/UInt8 | battery level in % |
| ImageSensor  | /drone/camera | Image | Image data | 

#### BatteryPublisher 

pubishes the status of the battery read from the tello status string
 
 ```
 $ rostopic echo /drone/battery
---
data: 66
---
data: 66
---
 ```


#### TwistSensor

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

#### available fields from the tello status string

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

### Servers

* KeepAliveServer

#### TODO

- preempted actions: drone should stop current command
- set speed


