## convertBAG

This program converts 3 types of data to rosbag files.
1. point cloud(.las)
2. gps and imu data(.txt)
3. image(.jpg) 

## Usage

    convertBAG inputFilesPath saveRosbagsPath
    
    just like
    
    ./convertBAG /home/user/inputfiles /home/user/BAGFiles
    
## Required dependencies
-ROS (https://www.ros.org/)

-libLAS (https://www.liblas.org/)

## Environment
Ubuntu 20.04 with ROS Noetic
    
## gps and imu data content
    [gpstime] [latitude] [longitude] [height] [pitch] [roll] [heading]
    
## about gps time
time correction needs to be done
