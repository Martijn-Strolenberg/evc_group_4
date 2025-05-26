# Group 4 Workshop 2 Documentation

### How to use custom messages
Stored in the folder 
``` workshop2_group4\src\jetson_camera\msg```

The custom message we use for the distorted and undistorted videos is called ```twovids.msg```

How to check if it is available
```
> catkin_make
> source devel/setup.bash
> rosmsg show twovids
```
it should come up with an output
```
sensor_msgs/CompressedImage raw_img
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string format
  uint8[] data
sensor_msgs/CompressedImage undist_img
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string format
  uint8[] data
```

Supposedly the way to call in code is:
<br>
msg.raw_img = ...
<br>
msg.undist_img = ...

