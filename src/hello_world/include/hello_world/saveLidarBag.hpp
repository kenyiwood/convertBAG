#include <iostream>
#include <string>

#include "ros/ros.h"

using namespace std;

struct LASContent {
  ros::Time stamp;
  double gpstime;

  double x;
  double y;
  double z;

  double intensity;
  double colorRed;
  double colorGreen;
  double colorBlue;
};

void readLASFile(string file, bool showDataDetail, string saveFileName,
                 double &eachStartTime, double &eachEndTime, string LeftOrRight,
                 int secondsBetweenIEAndROS, double &firstRecordTime);