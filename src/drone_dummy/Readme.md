# Drone Dummy

## Interface

### action servers

offers same actionservers as the drone package without sending commands to the drone.

### published topics

#### /drone/twist

Reads prerecorded telemetry data and replays it. Csv files are read from dir `data/telemetry`. See `scripts/drone_dummy_node.py` for details which file is used.

#### /drone/camera

Replays images from directory `data/camera`

## Usage

```
rosrun drone_dummy drone_dummy_node.py
```

### Cockpit

in the cockpit node, there are some additional 'dummy' nodes. edit `launch/mobrob.launch` file to control which are started

### run with docker

run drone_dummy & cockpit & additional dummy-nodes 

`docker compose  -f "docker-compose.yml" up -d --build ros_master cockpit cv`

#### linux

run xhost to enable docker connection to the hosts x-server
```
$ xhost +
access control disabled, clients can connect from any host
```

#### windows

install an x-server

