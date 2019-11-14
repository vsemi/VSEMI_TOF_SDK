VSEMI TOF SDK 1.3.2
Visionary Semiconductor Inc.
Nov 11, 2019

Introduction

   VSEMI ToF SDK is the development kit for Sentinel-V3 ToF camera, developed by Visionary Semiconductor Inc.

   Built with the Time-of-Flight (ToF) depth vision technology, Sentinel-V3 was designed for reliable depth sensing with high accuracy. 
   It was developed for easy and fast development and/or deployment in real applications, such as robotics, drones, and automotives. 
   As a ToF 3D camera, Sentinel-V3 does not require moving components and is thus robust and wear-free. 

Folder Structure

VSEMI_TOF_SDK
   |-docs:                          
   |   |-x86_amd64_ubuntu_16.04:    instructions of how to build and run samples on x86_amd64_ubuntu_16.04
   |   |-jetson_jetpack_3.0:        instructions of how to build and run samples on jetson_jetpack_3.0
   |-driver
   |   |-include:                   c++ header files of the ToF sensor driver
   |   |-lib:                       
   |      |-x64_ubuntu_16.04:       binary library of the ToF sensor driver for x64_ubuntu_16.04
   |      |-jetson_jetpack_3.0:     binary library of the ToF sensor driver for jetson_jetpack_3.x
   |-samples
   |   |-sample1:                   work with raw data with minimum_dependency
   |   |-sample2:                   work with depth map and point cloud in ROS environment
   |   |-sample3:                   work with depth map and point cloud using OpenCV and PCL
   |   |-sample4:                   work with depth map, point cloud and RGB camera in ROS environment
   |   |-sample5:                   work with depth map, point cloud and RGB camera using OpenCV and PCL
   |   |-sample6:                   work with depth map with OpenCV
   |-Readme.txt
   |-Release_notes.txt

