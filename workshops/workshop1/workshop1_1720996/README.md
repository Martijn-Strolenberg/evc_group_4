For this implementation of the workshop, I ran it through a docker container

Since I adjusted the .py file in windows, to run it on ubuntu, the  package dos2unix was required
so that the .py file can be read in linux.
Also the steps to run the file is


In the Workshop1_1720996 folder, the following commands were done in the case that assign1_sub and assign1_pub did not work
```
> catkin_make
> source devel/setup.bash
> dos2unix src/assign1_package/src/assign1_sub.py
> dos2unix src/assign1_package/src/assign1_pub.py
> roslaunch assign1_package Assign1_node.launch
```

** I Did not use the launch.sh file.