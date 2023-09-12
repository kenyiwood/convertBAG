#include <iostream>
#include <string>

using namespace std;

struct IEContent {
    double gpstime;
    double latitude;
    double longitude;
    double height;
    double VNorth;
    double VEast;
    double VUp;    

    double roll;
    double pitch;
    double heading;

    double SDNorth;
    double SDEast;
    double SDHeight;
};

void readIEFile(string file, vector<IEContent> &iecontent,
                int secondsBetweenIEAndROS);
void convertIEFileContentToBAG(vector<IEContent> iecontent, string saveFileName,
                               double eachStartTime, double eachEndTime);