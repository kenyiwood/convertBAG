#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <opencv4/opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/filesystem.hpp>
#include "hello_world/saveImageBag.hpp"
#include "hello_world/setProgressBar.hpp"

using namespace std;
using namespace cv;
const string ImageNodeTopicCam1 = "/cam_1/image_raw/compressed";
const string ImageNodeTopicCam7 = "/cam_7/image_raw/compressed";
const string frameIDImage = "mynteye_left_color_frame";
bool bagFileWritten = false;
int timeadd = 1;

void readImageFile(vector<ImageContent> imagecontent, string saveFileName, double eachStartTime, double eachEndTime)
{
    cout << "影像轉檔中...\n";
    ProgressBar *bar3 = setupProgressBarPartial("影像轉檔:", imagecontent.size());
    rosbag::Bag bag(saveFileName, rosbag::bagmode::Append);

    for (int i = 0; i < imagecontent.size(); i++)
    {
        if (imagecontent[i].gpstime >= eachStartTime && imagecontent[i].gpstime <= eachEndTime)
            saveImageBag(imagecontent[i], saveFileName, bag);

        bar3->Progressed(i + 1);
        timeadd++;
    }
    cout << "\n 影像輸出:" << saveFileName << "\n\n";
}

void saveImageBag(ImageContent imagecontent, string saveFileName, rosbag::Bag& bag)
{
    string imageFilePath = imagecontent.imageFilePath;
    string imageFileName = imagecontent.filename;
    double gpstime = imagecontent.gpstime;

    // 處理時間
    double time = gpstime;
    uint32_t sec = floor(time);
    uint32_t nsec = floor((time - sec) * 1000000000);
    ros::Time::init();
    ros::Time stamp(sec, nsec);

    sensor_msgs::CompressedImage sensorMsgsCImage;
    sensorMsgsCImage.header.stamp = stamp;
    sensorMsgsCImage.header.seq = 0;
    sensorMsgsCImage.header.frame_id = frameIDImage;
    sensorMsgsCImage.format = "jpg";

    Mat image = imread(imageFilePath);
    cv_bridge::CvImage image_bridge;
    std_msgs::Header header;
    header.stamp = stamp;
    header.seq = 0;
    header.frame_id = frameIDImage;

    image_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    image_bridge.toCompressedImageMsg(sensorMsgsCImage);

    if (imageFileName[0] == '1')
        bag.write(ImageNodeTopicCam1, stamp, sensorMsgsCImage);
    else
        bag.write(ImageNodeTopicCam7, stamp, sensorMsgsCImage);
}