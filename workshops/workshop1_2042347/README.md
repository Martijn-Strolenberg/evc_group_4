# Martijns personal workspace

## Docker build and run process
build a docker image from a docker file 
```powershell
docker build -t evc_ros:v1 .
```

run a docker container with name ```jetbot_cont1```
```powershell
docker run --rm -it -e DISPLAY=$DISPLAY --name jetbot_cont1 -p 11311:11311 -p 45100-45101:45100-45101 evc_ros:v1
```

Use ```docker ps``` to list all running containers. To open a new terminal inside a docker container called ```jetbot_cont1``` run:
```powershell
docker exec -it jetbot_cont1 /bin/bash
```

## ROS build and run process
1. navigate to the workspace directory
2. build all ros packages inside that workspace with ```catkin_make```
3. source the build packages with ```source devel/setup.bash```
4. launch (run) you ros application with ```roslaunch <package_name> <launch_file>```

I had some error that ros was trying to find python2/r package which doesn't exist because of the line end /r that windows adds. The fix I found was to install a package named dos2unix and then convert the file with the error using ```dos2unix src/workshop1_pkg/src/workshop1_node.py``` for example. That fixes the issue and now unix can find the python package.

### Usefull ROS commands
To list all topics:
```bash
rostopic list 
```
To observe the topics average publish rate (as well as min, max, std dev)
```bash
rostopic hz <topic> 
```

## WSL
To list your wsl distros:
```powershell
wsl --list --verbose
```
To start (and attach) your distro e.g. Ubuntu‐22.04
```powershell
wsl -d Ubuntu-22.04
```