# ROS Development Directory

This is the ROS directory for the development of all ROS nodes.

**ROS Distro**: melodic

View the following link for more details about creating ROS nodes and utilizing ROS packages: [ROS Wiki](https://wiki.ros.org/ROS)

## Caution

1. **File Alteration**: In order to run some of these nodes, you might need to alter some files.
2. **Implementation Details**: Currently implementing with `rospy` and `#!/usr/bin/env python2` (roscpp not tested).
3. **cv_bridge Issue**:
   - Due to Jetson quirks, when installing `cv_bridge`, it gets installed in folder `/usr/include/opencv4`.
   - When running `catkin_make`, the following error occurs:
     ```
     Project 'cv_bridge' specifies '/usr/include/opencv' as an include dir, which is not found.
     ```
   - Solution: You can install `cv_bridge` via git, build it yourself, and link its location to `/usr/include/opencv`, but the easiest fix is:
     - Go to: `/opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake`
     - Change the line:
       ```
       set(_include_dirs "include;/usr/include;/usr/include/opencv")
       ```
       to:
       ```
       set(_include_dirs "include;/usr/include;/usr/include/opencv4")
       ```
     - [Credits](https://github.com/ros-perception/vision_opencv/issues/345)
4. **Pink Tint in Image Viewer**: This can be solved by following these instructions:
   ```bash
   wget https://www.waveshare.com/w/upload/e/eb/Camera_overrides.tar.gz
   tar zxvf Camera_overrides.tar.gz
   sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/
   sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
   sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
   ```
     - [Credits](https://jonathantse.medium.com/fix-pink-tint-on-jetson-nano-wide-angle-camera-a8ce5fbd797f)