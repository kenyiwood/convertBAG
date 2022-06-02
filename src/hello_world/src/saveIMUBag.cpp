#include <string>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include "hello_world/saveIMUBag.hpp"

using namespace std;
const double DegreeConvertToRadian = M_PI / 180;
const string IMUNodeTopic = "/pwrpak/pose";
const string frameIDIMU = "pwrpak";

void saveIMUBag(rosbag::Bag &bag, ros::Time stamp, geometry_msgs::PoseStamped pose)
{
    bag.write(IMUNodeTopic, stamp, pose);
}

inline double SIGN(double x)
{
    return (x >= 0.0) ? +1.0 : -1.0;
}

inline double NORM(double a, double b, double c, double d)
{
    return sqrt(a * a + b * b + c * c + d * d);
}

double *ConvertRotationMatrixToQuaternion(double rolls, double pitchs, double heading)
{
    static double Quaternion[4];  // quaternion = [w, x, y, z]'
    double yaw = heading * -1 * DegreeConvertToRadian;  // yaw = -heading
    double roll = rolls * DegreeConvertToRadian;
    double pitch = pitchs * DegreeConvertToRadian;
    double rotationMatrix[3][3];

    rotationMatrix[0][0] = cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll);
    rotationMatrix[0][1] = -1 * sin(yaw) * cos(pitch);
    rotationMatrix[0][2] = cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);

    rotationMatrix[1][0] = sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);
    rotationMatrix[1][1] = cos(yaw) * cos(pitch);
    rotationMatrix[1][2] = sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll);

    rotationMatrix[2][0] = cos(pitch) * sin(roll);
    rotationMatrix[2][1] = sin(pitch);
    rotationMatrix[2][2] = cos(pitch) * cos(roll);

    double r11 = rotationMatrix[0][0], r12 = rotationMatrix[0][1], r13 = rotationMatrix[0][2];
    double r21 = rotationMatrix[1][0], r22 = rotationMatrix[1][1], r23 = rotationMatrix[1][2];
    double r31 = rotationMatrix[2][0], r32 = rotationMatrix[2][1], r33 = rotationMatrix[2][2];

    double q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
    double q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
    double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;

    if (q0 < 0.0)
        q0 = 0.0;

    if (q1 < 0.0)
        q1 = 0.0;

    if (q2 < 0.0)
        q2 = 0.0;

    if (q3 < 0.0)
        q3 = 0.0;

    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);

    if (q0 >= q1 && q0 >= q2 && q0 >= q3)
    {
        q0 *= +1.0;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    }
    else if (q1 >= q0 && q1 >= q2 && q1 >= q3)
    {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    }
    else if (q2 >= q0 && q2 >= q1 && q2 >= q3)
    {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0;
        q3 *= SIGN(r32 + r23);
    }
    else if (q3 >= q0 && q3 >= q1 && q3 >= q2)
    {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0;
    }
    else
    {
        cout << "coding error\n";
    }
    float r = NORM(q0, q1, q2, q3);
    q0 /= r;
    q1 /= r;
    q2 /= r;
    q3 /= r;

    Quaternion[0] = q0;
    Quaternion[1] = q1;
    Quaternion[2] = q2;
    Quaternion[3] = q3;

    return Quaternion;
}

void convertIEFileContentToBAG(vector<IEContent> iecontent, string saveFileName)
{
    // write geometry_msgs::PoseStamped
    if (iecontent.size() > 0)
    {
        rosbag::Bag bag(saveFileName, rosbag::bagmode::Write);

        for (int i = 0; i < iecontent.size(); i++)
        {
            double time = iecontent[i].gpstime;
            uint32_t sec = floor(time);
            uint32_t nsec = floor((time - sec) * 1000000000);
            ros::Time::init();
            ros::Time stamp(sec, nsec);
            geometry_msgs::PoseStamped pose;

            pose.header.stamp = stamp;
            pose.header.seq = 0;
            pose.header.frame_id = frameIDIMU;
            double x = iecontent[i].longitude;
            double y = iecontent[i].latitude;
            double z = iecontent[i].height;

            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;

            double *Quaternion = ConvertRotationMatrixToQuaternion(iecontent[i].roll, iecontent[i].pitch, iecontent[i].heading);

            pose.pose.orientation.w = Quaternion[0];
            pose.pose.orientation.x = Quaternion[1];
            pose.pose.orientation.y = Quaternion[2];
            pose.pose.orientation.z = Quaternion[3];
            saveIMUBag(bag, stamp, pose);
        }
        bag.close();
        cout << "檔案輸出:" << saveFileName << "\n\n";
    }
}

void readIEFile(string file, bool showDataDetail, string saveFileName)
{
    vector<IEContent> iecontent;
    // vector<double> times, latitudes, longitudes, heights, rolls, pitchs, headings;
    ifstream ifs(file, std::ios::in);
    if (!ifs.is_open())
        cout << file << "檔案開啟失敗!\n";
    else
    {
        cout << file << "定位檔轉換\n";
        double gpstime, latitude, longitude, height, VNorth, VEast, VUp, roll, pitch, heading, SDNorth, SDEast, SDHeight;
        while (ifs >> gpstime >> latitude >> longitude >> height >> VNorth >> VEast >> VUp >> roll >> pitch >> heading >> SDNorth >> SDEast >> SDHeight)
        {
            IEContent oneData;
            oneData.gpstime = gpstime;
            oneData.latitude = latitude;
            oneData.longitude = longitude;
            oneData.height = height;
            oneData.pitch = pitch;
            oneData.roll = roll;
            oneData.heading = heading;
            iecontent.push_back(oneData);
        }
    }
    ifs.close();
    convertIEFileContentToBAG(iecontent, saveFileName);

    if (showDataDetail)
    {
        for (int i = 0; i < iecontent.size(); i++)
        {
            cout << iecontent[i].gpstime << "  "
                 << iecontent[i].latitude << "  "
                 << iecontent[i].longitude << "  "
                 << iecontent[i].height << "  "
                 << iecontent[i].roll << "  "
                 << iecontent[i].pitch << "  "
                 << iecontent[i].heading << endl;
        }
    }
}