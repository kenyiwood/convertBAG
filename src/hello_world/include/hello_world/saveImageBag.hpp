#include <iostream>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include "hello_world/progress_bar.hpp"

using namespace std;

struct ImageContent
{
    ros::Time stamp;
    double gpstime;
    string imageFilePath;
    string filename;
};

void readImageFile(vector<ImageContent> imagecontent, string saveFileDirectorysaveFileName, double eachStartTime, double eachEndTime);
void saveImageBag(ImageContent imagecontent, string saveFileName,
                  rosbag::Bag& bag);