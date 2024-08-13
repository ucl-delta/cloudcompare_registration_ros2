# ROS2 Livox CloudCompare

This package attempts to integrate cloudcompare with ros2 and a livox stream for registration

## Cloud Compare Docker

This project is used for communicating with cloudcompare 
- https://github.com/CloudCompare/CloudComPy

Since the setup is a bit challenging, we take the docker container build method and transfer it over into this repository. 
This docker container acts as a first stage for the ros2 node we intend to build. 

Note that the dockerfile actually utilises this repo: https://gitlab.com/openfields1/CloudComPy with the branch set to CloudComPy_docker-20230216. Therefore not all of the latest API exists. 

## Usage

Refer to make file for specific instructions. 

```bash
make # Builds the container
make run # Builds and runs the container, running the registration server
make run_bash #Builds and runs container, puts you inside the container
make test # Builds and runs the colcon tests in the container. 
```

If running with other ros nodes in containers or bare-metal, the default arguments for `make run` should allow for shared memory transport etc so hopefully should work. 

## API

This server requests the `Registration.srv` service type within `pointcloud_registration_msgs`

```
# Reference Name for this pointcloud
string name

# Pointcloud
sensor_msgs/PointCloud2 pointcloud

# Initial Transform of the pointcloud before registration
geometry_msgs/TransformStamped initial_transform

# Clip distance threshold before applying ICP
float64 clip_distance_threshold

# Two 3D points defining the top and bottom most corners to crop out
geometry_msgs/Point[] crop_corner 
---
# Returns a code to show if it worked
uint32 return_code

# Returns the transform of the object if successful
geometry_msgs/TransformStamped transform
```
