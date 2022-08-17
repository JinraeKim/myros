# myros
My ROS recipes


## Image & depth data streaming and collecting

Here, it is assumed to use Realsense camera (e.g. D435).

The following instruction is based on 
[realsense-ros wrapper](https://github.com/IntelRealSense/realsense-ros).

First, install the Realsense ROS wrapper.


### Stream RGB and other data
```
roslaunch realsense2_camera rs_camera.launch
```

Note: obviously, RGB and depth sensors are mounted slightly different positions and it causes the frame difference.
To resolve this issue, use the `align_depth:=true` option described as below.

Other options: `color_fps`, `depth_fps`, etc.

**NOTICE: high fps may stop the computer. It is recommended you to designate low fps as below.**

### Stream RGB and aligned depth data
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true color_fps:=10 depth_fps:=10
```


## Save data to ROS bag files
The following instruction is based on 
[rosbag cli page](http://wiki.ros.org/rosbag/Commandline).

### Recipes
```
rosbag record --duration=10 -o record_t1.bag /camera/color/image_raw /camera/aligned_depth_to_color/image_rect_raw
```
