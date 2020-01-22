# Kinect2 Bridge

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Description

This is a bridge between [libfreenect2](https://github.com/OpenKinect/libfreenect2) and ROS.

### Highlights

- delivers up to 30 frames per second on non high end hardware
- delivers up to 30 frames per second over gigabit ethernet
- support for compressed image transport
- utilizes multiple cores and uses special OpenCL based implementation of the depth registration

## Dependencies

- ROS Hydro/Indigo
- OpenCV
- [libfreenect2](https://github.com/OpenKinect/libfreenect2)

*for the ROS packages look at the package.xml*

## First steps

For the depth registration the camera intrinsics and extrinsics need to be known. The program reads in the values from the `data/<serialnumber>` folder. For each new sensor you need to add a subfolder with the serial number of the device as the folder name. In this folder you need to provide 3 yaml files with the intrinsics and extrinsics. These files can be created by the `kinect2_calibration` tool (or you can copy the files provided in one of the other folders, but results can be sub optimal). The device serial number is shown when `kinect2_bridge` or `Protonect` from libfreenect2 is started, it also appears in `dmesg` when you connect the sensor. [More information on calibration](../kinect2_calibration#calibrating-the-kinect-one).

When `kinect2_bridge` is running you can use the `registration_viewer` to display the images or point cloud: `rosrun registration_viewer viewer -kinect2 -image` or `rosrun registration_viewer viewer -kinect2 -cloud`.

## Topics

### Depth Topics

###### Raw depth image
```
/kinect2/depth/camera_info
/kinect2/depth/image
/kinect2/depth/image/compressedDepth
```

###### Rectified depth image
```
/kinect2/depth_rect/camera_info
/kinect2/depth_rect/image
/kinect2/depth_rect/image/compressedDepth
```

###### Depth image registered to low resolution image (960x540)
```
/kinect2/depth_lowres/camera_info
/kinect2/depth_lowres/image
/kinect2/depth_lowres/image/compressedDepth
```

###### Depth image registered to high resolution image
```
/kinect2/depth_highres/camera_info
/kinect2/depth_highres/image
/kinect2/depth_highres/image/compressedDepth
```

### Infrared Topics

###### Raw ir image
```
/kinect2/ir/camera_info
/kinect2/ir/image
/kinect2/ir/image/compressed
```

###### Rectified ir image
```
/kinect2/ir_rect/camera_info
/kinect2/ir_rect/image
/kinect2/ir_rect/image/compressed
```

### Mono Topics

###### Raw mono image
```
/kinect2/mono/camera_info
/kinect2/mono/image
/kinect2/mono/image/compressed
```

###### Rectified mono image
```
/kinect2/mono_rect/camera_info
/kinect2/mono_rect/image
/kinect2/mono_rect/image/compressed
```

###### Mono image in low resolution (960x540)
```
/kinect2/mono_lowres/camera_info
/kinect2/mono_lowres/image
/kinect2/mono_lowres/image/compressed
```

### Color Topics

###### Raw color image
```
/kinect2/rgb/camera_info
/kinect2/rgb/image
/kinect2/rgb/image/compressed
```

###### Rectified color image
```
/kinect2/rgb_rect/camera_info
/kinect2/rgb_rect/image
/kinect2/rgb_rect/image/compressed
```

###### Color image in low resolution (960x540)
```
/kinect2/rgb_lowres/camera_info
/kinect2/rgb_lowres/image
/kinect2/rgb_lowres/image/compressed
```

### Point cloud Topics
*Only available if `kinect2_bridge.launch` is launched.*

```
/kinect2/depth_lowres/points
/kinect2/depth_highres/points
```

## Notes

- Images from the same frame have the same timestamp. Using the `message_filters::sync_policies::ExactTime` policy is recommended.

## Usage

```
kinect2_bridge [_options:=value]
_base_name:=<string>
    default: kinect2
    info:    set base name for all topics
_sensor:=<string>
    default:
    info:    serial of the sensor to use
_fps_limit:=<double>
    default: -1.0
    info:    limit the frames per second
_calib_path:=<string>
    default: /home/wiedemeyer/work/src/iai_kinect2/kinect2_bridge/data/
    info:    path to the calibration files
_use_png:=<bool>
    default: false
    info:    Use PNG compression instead of TIFF
_jpeg_quality:=<int>
    default: 90
    info:    JPEG quality level from 0 to 100
_png_level:=<int>
    default: 1
    info:    PNG compression level from 0 to 9
_depth_method:=<string>
    default: opencl
    info:    Use specific depth processing: default, cpu, opengl, opencl
_depth_device:=<int>
    default: -1
    info:    openCL device to use for depth processing
_reg_method:=<string>
    default: opencl
    info:    Use specific depth registration: default, cpu, opencl
_reg_devive:=<int>
    default: -1
    info:    openCL device to use for depth registration
_max_depth:=<double>
    default: 12.0
    info:    max depth value
_min_depth:=<double>
    default: 0.1
    info:    min depth value
_queue_size:=<int>
    default: 2
    info:    queue size of publisher
_bilateral_filter:=<bool>
    default: true
    info:    enable bilateral filtering of depth images
_edge_aware_filter:=<bool>
    default: true
    info:    enable edge aware filtering of depth images
_publish_tf:=<bool>
    default: false
    info:    publish static tf transforms for camera
_base_name_tf:=<string>
    default: as base_name
    info:    base name for the tf frames
_worker_threads:=<int>
    default: 4
    info:    number of threads used for processing the images
```

## Key bindings

Terminal:
- `CRTL`+`c`: Quit

