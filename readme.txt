This program converts 3 types of data to rosbag files.
1. point cloud(.las)
2. gps and imu data(.txt)
3. image(.jpg) 

To run this program use:
    convertBAG inputFilesPath saveRosbagsPath

just like
    ./convertBAG /home/user/inputfiles /home/user/BAGFiles
    
## gps and imu data content:
    [gpstime] [latitude] [longitude] [height] [pitch] [roll] [heading]
    
## about gps time
some correction needs to be done
