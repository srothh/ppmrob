# Cockpit

Visualization module, subscribes available topics and tries to paint a picture about the drones state.

## subscribed topics:

```
$ rosnode info /cockpit
...
Subscriptions: 
 * /TransformActionServer/goal [unknown type]
 * /battery/return_signal [unknown type]
 * /drone/battery [unknown type]
 * /drone/camera [unknown type]
 * /drone/twist [unknown type]
 * /move/goal [unknown type]
 * /odometry/return_signal [unknown type]
 * /cv/victim [unknown type]

...
```

## display on linux 

first run this on local machine to allow all users to connect to the X-Server
```
$ xhost +
```
see: https://stackoverflow.com/questions/49169055/docker-tkinter-tclerror-couldnt-connect-to-display

NOTE: on windows you may have to install an X-Server
