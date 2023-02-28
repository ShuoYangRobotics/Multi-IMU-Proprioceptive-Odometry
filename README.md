# Multi-IMU-Proprioceptive-Odometry

This repo includes an example code of the multi-IMU proprioceptive odometry algorithm described in the paper:

Multi-IMU Proprioceptive Odometry For Legged Robots

## Installation

The only dependency is the Casadi library (V3.5.5). It can be downloaded from Casadi [official website](https://web.casadi.org/get/). Choose the version that matches your OS and Matlab version. Download the file and extract its content to the folder `casadi`.

## Usage

Download dataset from [Google drive folder](https://drive.google.com/drive/folders/174qgyZykdeqs4t8N7NnVp_i18W1NxHYY?usp=sharing).

In the script `scripts/RUN_MAIN.m`, set variable `file_path` to the path of the dataset rosbag file. Then run the script.