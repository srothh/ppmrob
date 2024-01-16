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
To create custom nodes, recreate the folder structure for your node, edit the variables PACKAGE_NAME and LAUNCH_FILE in the Dockerfile
and add the node/service to the docker-compose.yml file.