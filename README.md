# LOL: Lidar-only  Odometry  and  Localization  in  3D  point  cloud  maps
### Supplementary material for our ICRA 2020 paper 


**Abstract -** In this paper we deal with the problem of odom-
etry  and  localization  for  Lidar-equipped  vehicles  driving  in
urban  environments,  where  a  premade  target  map  exists  to
localize  against.  In  our  problem  formulation,  to  correct  the
accumulated drift of the Lidar-only odometry we apply a place
recognition  method  to  detect  geometrically  similar  locations
between the online 3D point cloud and the a priori offline map.
In  the  proposed  system,  we  integrate  a  state-of-the-art  Lidar-
only  odometry  algorithm  with  a  recently  proposed  3D  point
segment matching method by complementing their advantages.
Also,  we  propose  additional  enhancements  in  order  to  reduce
the  number  of  false  matches  between  the  online  point  cloud
and   the   target   map,   and   to   refine   the   position   estimation
error whenever a good match is detected. We demonstrate the
utility of the proposed LOL system on several Kitti datasets of
different  lengths  and  environments,  where  the  relocalization
accuracy  and  the  precision  of  the  vehicle’s  trajectory  were
significantly  improved  in  every  case,  while  still  being  able  to
maintain  real-time  performance.

[arXiv](https://arxiv.org/pdf/2007.01595.pdf) | [Video](https://www.youtube.com/watch?v=ektGb5SQGRM&feature=youtu.be)


<div align="center">
    <img src="https://github.com/RozDavid/LOL/blob/master/resources/LOL_gif.gif" width = 80% >
</div>

**Advances to the state-of-the-art:**
  - We present a novel Lidar-only odometer and Localization system by integrating and complementing the advantages of two state of the algorithms. 
  - We propose a set of enhancements: (i) a RANSAC-based geometrical verification to reduce the number of false 
    matches between the online point cloud and the offline map; and (ii) a fine-grained ICP alignment to refine the relocalization accuracy whenever a good match is detected.
  - Here we publicly release the source code of the proposed system with supplementary prepared datasets to test.
  
For any code-related or other questions open an issue [here](https://github.com/RozDavid/LOL/issues). If you found this work helpful for your research, please cite our paper:

```text
@inproceedings{lol_3dlocalization, 
	title={{LOL: Lidar-only Odometry and Localization in 3D point cloud maps}}, 
	author={Rozenberszki, Dávid and Majdik, András}, 
	booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)}, 
	year={2020},
        organization={IEEE} 
}
```  

### Installation
**Ubuntu 64-bit 16.04. ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)**

Install the dependencies:

```sh
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install python-wstool python-catkin-pkg doxygen python3-pip python3-dev python-virtualenv dh-autoreconf python-catkin-tools
```

Now create the Catkin Environment:

```sh
$ mkdir -p ~/Catkin/LOL/src
$ cd ~/Catkin/LOL
$ catkin init
$ catkin config --merge-devel
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
And clone the project:
```sh
$ git clone https://github.com/RozDavid/LOL.git
$ wstool init
$ wstool merge segmap/dependencies.rosinstall
$ wstool update
```

**The CNN descriptors were made in Tensorflow, for compiling the whole package and using the localization function with learning based descriptors one needs to [install](https://www.tensorflow.org/install/pip) for Ubuntu 16.04**

Finally build the packages with:
```sh
$ cd ~/Catkin/LOL
$ catkin build tensorflow_ros_cpp
$ catkin build segmapper
$ catkin build loam_velodyne
```

###Rosbag Examples

Download the provided map resources to your machine from [here](https://www.dropbox.com/sh/c81dms7qbpp5936/AAAtgIygoivnyq5hcPGDwhtUa?dl=0) and save them anywhere in your machine. The Rosbags for the examples could be downloaded from the original Kitti dataset [website](http://www.cvlibs.net/datasets/kitti/raw_data.php?type=residential), you just need to strip other sensor measurement and */tf* topic from it to run correctly. 

Then modify the folowing *launch* and *yaml* and set the path for downloaded dataset files
```sh
$ ~/Catkin/LOL/segmap/segmapper/launch/kitti/cnn_kitti_loam_segmap.yaml
$ ~/Catkin/LOL/segmap/segmapper/launch/kitti/cnn_loam_segmap.launch
$ ~/Catkin/LOL/segmap/segmapper/launch/kitti/kitti_loam_segmap.launch
$ ~/Catkin/LOL/segmap/segmapper/launch/kitti/kitti_localization.yaml
```

### Running the Examples
##### Run Localization with Eigen features

```roslaunch segmapper kitti_loam_segmap.launch```


##### Run Localization with CNN features

```roslaunch segmapper cnn_loam_segmam.launch```

### Acknowledgement
This source code and the resulting paper is highly dependent and mostly based on two amazing state-of-the art algorithms. 
The Odometry is calculated by the [LOAM](https://github.com/laboshinl/loam_velodyne), while the segmentation, feature detection and matching is based on the [SegMap](https://github.com/ethz-asl/segmap) algorithm. Without these works this paper wouldn't be able to exist. 

The research reported in this paper was supported by the Hungarian Scientific Research Fund (No. NKFIH OTKA KH-126513) and by the project: Exploring the Mathematical Foundations of Artificial Intelligence 2018-1.2.1-NKP-00008.
