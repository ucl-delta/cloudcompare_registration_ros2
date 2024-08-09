# ROS2 Livox CloudCompare

This package attempts to integrate cloudcompare with ros2 and a livox stream for registration

## Cloud Compare

This project is used for communicating with cloudcompare 
- https://github.com/CloudCompare/CloudComPy

Since the setup is a bit challenging, we take the docker container build method and transfer it over into this repository. 
This docker container acts as a first stage for the ros2 node we intend to build. 