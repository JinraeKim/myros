# myros
My ROS recipes


# Image, depth, etc.
## Tested cameras
- Realsense D435
- Realsense D455

The following instruction is highly based on 
[realsense-ros wrapper](https://github.com/IntelRealSense/realsense-ros).

First, install the Realsense ROS wrapper.

## Image & depth data streaming and collecting
### Stream RGB and other data
```
roslaunch realsense2_camera rs_camera.launch
```

Note: obviously, RGB and depth sensors are mounted slightly different positions and it causes the frame difference.
To resolve this issue, use the `align_depth:=true` option described as below.

Other options: `color_fps`, `depth_fps`, etc.

<!-- **NOTICE: high fps may stop the computer. It is recommended you to designate low fps as below.** -->

### Stream RGB and aligned depth data
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

### Load and publish an image file
```
python3 load_and_publish_image.py --image my_image.png
```


## Save images to ROS bag files and png files
The following instruction is based on 
[rosbag cli page](http://wiki.ros.org/rosbag/Commandline).

### Rename topics inside a rosbag (before running the script, inspect and modify the code!!!)
```
python3 rosbag_rename.py --bag myrosbag.bag
```

### Recipes for ROS image topics to ROS bag files
```
rosbag record --duration=10 -o record_t1.bag /camera/color/image_raw /camera/aligned_depth_to_color/image_raw
```

### Recipes for ROS bag files to png files
```
python bag_to_images.py --bag <my_bag.bag> --camera <my_camera>
```
For example,
```
python bag_to_images.py --bag a.bag --camera /camera
```

Note: if you use python3 with cv_bridge (and disabled cv_bridge for python2),
you may need to run by the following code:

```
python3 bag_to_images.py --bag a.bag --camera /camera
```

Also, if you encounter some errors, e.g., 
```
ModuleNotFoundError: No module named 'Cryptodome'
```
then install those as `python3 -m pip install pycryptodomex`.


## Recipes for pointcloud
### Conversion from color and depth images to pointcloud
Based on [depth_image_proc](http://wiki.ros.org/depth_image_proc#depth_image_proc.2Fpoint_cloud_xyz).

```
roslaunch point_cloud_xyzrgb.launch
```

### Read pointcloud corresponding to pixel indices
Based on [point_cloud2.py](https://docs.ros.org/en/api/sensor_msgs/html/point__cloud2_8py_source.html).

First, you may need to launch realsense camera as below.

```
roslaunch realsense2_camera rs_camera.launch align_depth:=true image_fps:=5 depth_fps:=5 filters:=pointcloud
```

And run `./pointcloud_manipulator.py`.
It will collect only specific points corresponding to given indices (see the file for more details).


## ROS app with opencv
### RGB image processing
You can find an example of drawing a circle on the streamed camera image in `./image_converter.py`.

### Synchronise multiple topics
You can Synchronise multiple ROS topics by using `message_filters.TimeSynchronizer`.
For more details, see `image_depth_synchroniser.py`.


## Hardware stuff

### Weird behaviours...?
- I needed to connect Realsense camera through USB 3 port. Sometimes, the USB port would be different from what you guessed (e.g., I used blue port, which is supposed to be USB 3.0, but it was 2.1. And I changed the port to white one, which is supposed to be USB 2.1, but the warning was suppressed and worked perfectly...!)


# tf (Transform)
## Visualise tf tree recipe
```
rosrun rqt_tf_tree rqt_tf_tree
```

## Trouble shootings
### It is not able to visualise PointCloud2 on rviz
#### Error message
```
Transform [sender=unknown_publisher]
For frame [camera_color_optical_frame]: Fixed Frame [map] does not exist
```

#### Reason
tf does not have a transform between `camera_link` and `map`.
It often occurs when you don't run any SLAM algorithms and try to run RGB-D camera only.

#### Solution
In the case of that you're running RGB-D camera only,
change the `Fixed Frame` in `Global Options` on `rviz` to `camera_link`.
