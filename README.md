# ppmrob
## Initial docker setup
To run use

``
docker compose up --build
``

To exit send `Ctrl+C` or run

``
docker compose down
``

## Development

    # in catkin workspace (i.e., ppmrob)
    catkin_make
    source devel/setup.bash


When successful you should be able to run, e.g., `roscd battery`.

### Custom nodes


~~To create custom nodes, recreate the folder structure for your node, edit the variables PACKAGE_NAME (name of the package/node you want to create) and LAUNCH_FILE(name of the launch file in your directory for this node) in the Dockerfile
and add the node/service to the docker-compose.yml file.~~ TODO workflow will be added shortly
