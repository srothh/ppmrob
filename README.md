# ppmrob
## Initial docker setup
### To run use:
``
docker compose up --build
``

This defaults to 1 victim, change `NUMBER_OF_VICTIMS=value` in `docker_planner_vars.env` for different number of victims.

### To exit
Send `Ctrl+C` or run:

``
docker compose down
``

## Development

    # in catkin workspace (i.e., ppmrob)
    catkin_make # compile
    source devel/setup.bash # update the workspace environment


When successful you should be able to run, e.g., `roscd <package>`.

### Creating a ROS package
For a package to be considered a catkin package it must meet a few requirements:
1.  catkin compliant `package.xml`
1. `CMakeLists.txt` which uses catkin
1. have its own folder

        workspace_folder/        -- WORKSPACE
          src/                   -- SOURCE SPACE
            CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
            package_1/
              CMakeLists.txt     -- CMakeLists.txt file for package_1
              package.xml        -- Package manifest for package_1
            ...
            package_n/
              CMakeLists.txt     -- CMakeLists.txt file for package_n
              package.xml        -- Package manifest for package_n

Project file structure as a tree (find more info [here](https://www.yahboom.net/public/upload/upload-html/1640334504/7.2%20Introduction%20of%20project%20files.html)):  
![catkin workspace file system](https://github.com/srothh/ppmrob/assets/128387629/88483141-cafa-4f00-95af-474e443ee353)

### Custom nodes
1. [CreatingPackage](http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage)
2. Add `Dockerfile` to the package folder
3. Add the node/service to the `docker-compose.yml` file

# [Initial commit](https://github.com/srothh/ppmrob/commit/a84e313148c968950890c279be86650bc3b27f8c)
