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

## Nodes

### Drone

#### Commands

- takeoff 

Auto takeoff.

- land

Auto landing.

- streamon

Enable video stream.
streamoff Disable video stream.
emergency Stop motors immediately.
up x Ascend to “x” cm.
x = 20-500
down x down “x” Descend to “x” cm.
x = 20-500
left x Fly left for “x” cm.
“x” = 20-500
right x Fly right for “x” cm.
“x” = 20-500
forward x Fly forward for “x” cm.
“x” = 20-500
back x Fly backward for “x” cm.
“x” = 20-500
cw x Rotate “x” degrees clockwise.
“x” = 1-360
ccw x Rotate “x” degrees counterclockwise.
“x” = 1-360
flip x Flip in “x” direction.
“l” = left
“r” = right
“f” = forward
“b” = back
ok / error
Fly to “x” “y” “z” at “speed” (cm/s).
go x y z speed

#### Status

#### Video stream
