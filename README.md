# 📝 Image Dataset to ROS Bag #
***
This ROS tool aims to help create ros bag data from the original image datasets, that can be used directly on ROS packages.
I created this tool after some attempts to download OSS stereo image datasets, that are originally stored in ROS bag representation to test algorithms written in ROS package form, but could not find a really good source for my use.
More than that, sadly, famous stereo image datasets like [Middlebury dataset](https://vision.middlebury.edu/stereo/) are not provided in ROS bag form for plug-and-play use :(.

## :tada: TODO
***

- [x] mono image dataset to rosbag
- [x] stereo image dataset to rosbag, with option to conduct rectification from camera info

## 🎛  Dependencies
***
- tested on Ubuntu 20.04, but should work with older distributions.
- ROS1
- OpenCV

## 🔨 How to Build ##
***
Better to build with catkin or colcon

## :running: How to Run ##
***

- mono image dataset to rosbag: The following metadata files need to be prepared:
  + calibration file stored in yaml form. [HERE](./data/samples/left.yaml) is an example.
  +  metadata file with names of images in the dataset for the left and right lens. [HERE](./data/samples/left.txt) is an example.

An example for the launch file is also provided [HERE](./launch/mono_bag_converter.launch)

- stereo image dataset to rosbag: The following metadata files need to be prepared:
  + calibration files stored in yaml form for left and right lens.
  + metadata file with names of images in the dataset for the left and right lens. [HERE](./data/samples/left.txt) is an example.

An example for the launch file is also provided [HERE](./launch/stereo_bag_converter.launch)

***

Please check sample launch files to know how to modify parameters. If not specified, the output rosbag will be saved in ~/.ros/output.bag .

*I tested this package on [DrivingStereo dataset](https://drivingstereo-dataset.github.io/).*

![driving stereo](./data/images/samples.gif)

## :gem: References ##
***
- [ROS image_pipeline](https://github.com/ros-perception/image_pipeline)
- [DrivingStereo dataset](https://drivingstereo-dataset.github.io/)
