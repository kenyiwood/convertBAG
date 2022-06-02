#include <iostream>
#include <string>
#include <ros/ros.h>
#include "hello_world/progress_bar.hpp"

using namespace std;

struct ImageContent
{
    ros::Time stamp;
    double gpstime;
    string imageFilePath;
};

void readImageFile(vector<ImageContent> imagecontent, string saveFileDirectory);
void saveImageBag(ImageContent imagecontent, string saveFileName);