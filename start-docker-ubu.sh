# This file is used by Karsten in order to startup his docker container (I am lazy and forgetfull)
#
#
#
#
echo "Running start-docker-ubu.sh"

xhost +local:docker

sudo docker build -t ros:custom .

sudo docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /home/karsten/evc_group_4:/home/ubuntu/ros_ws \
  ros:custom