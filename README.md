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
roslaunch realsense2_camera rs_camera.launch align_depth:=true color_fps:=5 depth_fps:=5
```


## Save images to ROS bag files and png files
The following instruction is based on 
[rosbag cli page](http://wiki.ros.org/rosbag/Commandline).

### Recipes for ROS image topics to ROS bag files
```
rosbag record --duration=10 -o record_t1.bag /camera/color/image_raw /camera/aligned_depth_to_color/image_raw
```

### Recipes for ROS bag files to png files
```
python bag_to_images.py record_t1
```


## ROS app with opencv
### RGB image processing
You can find an example of drawing a circle on the streamed camera image in `./image_converter.py`.

### Synchronise multiple topics
You can Synchronise multiple ROS topics by using `message_filters.TimeSynchronizer`.
For more details, see `image_depth_synchroniser.py`.
