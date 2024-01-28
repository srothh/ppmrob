# Tello

## Actions

| action  | argument  |   |
|---|---|---|
| launch  | bool order  | true: takeoff <br> false: land  |
| rotate |  int bearing  | negative: ccw <br> positive cw  |
| x |  int distance  | negative: back <br> positive forward  |
| y |  int distance  | negative: left <br> positive right  |
| z |  int distance  | negative: down <br> positive up  |
|  command | string  | execute command  | 

## Sensors

published state

| publisher  | | topic | comment |
|---|---|---|
| TelemetrySensor  | telemetry  | all state fields |
| BatteryPublisher  | battery | battery status |

### BatteryPublisher 

queries actively the status of the battery (command 'battery?') to prevent safety hutdown after 15 sec. NOT WORKING

### TelemetrySensor

contains all fields from the status string broadcasted by the drone

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


### TODO

- preempted actions: drone should stop current command
- set speed, 
- streaming server, streamof, streamoff




