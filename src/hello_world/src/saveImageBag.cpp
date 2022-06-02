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

using namespace std;
using namespace cv;
const string ImageNodeTopic = "/mynteye/left/image_color/compressed/undistorted";
const string frameIDImage = "mynteye_left_color_frame";
int howManyPicturesInABagFile = 5;
bool bagFileWritten = false;
int timeadd = 1;

void readImageFile(vector<ImageContent> imagecontent, string saveFileDirectory)
{
    string saveSubFolder;
    string saveBagFile;

    cout << "影像轉檔中...\n";
    ProgressBar *bar3 = new ProgressBar(imagecontent.size(), "影像轉檔:");
    if (imagecontent.size() < 100)
        bar3->SetFrequencyUpdate(imagecontent.size());
    else
        bar3->SetFrequencyUpdate(100);
    bar3->SetStyle("|", "-");

    for (int i = 0; i < imagecontent.size(); i++)
    {
        saveSubFolder = saveFileDirectory + "/image";
        saveBagFile = saveSubFolder + "/" + to_string(i / howManyPicturesInABagFile) + ".bag";

        if (!boost::filesystem::is_directory(saveSubFolder))
            boost::filesystem::create_directories(saveSubFolder);

        if (i % howManyPicturesInABagFile == 0)
            bagFileWritten = false;

        bar3->Progressed(i + 1);

        saveImageBag(imagecontent[i], saveBagFile);
        timeadd++;
    }
    cout << "\n\n";
}

void saveImageBag(ImageContent imagecontent, string saveFileName)
{
    string imageFilePath = imagecontent.imageFilePath;

    // 處理時間
    double time = 1651548673.123456789 + timeadd;
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
    rosbag::Bag bag;
    if (!bagFileWritten)
    {
        bag.open(saveFileName, rosbag::bagmode::Write);
        bagFileWritten = true;
    }
    else
        bag.open(saveFileName, rosbag::bagmode::Append);

    bag.write(ImageNodeTopic, stamp, sensorMsgsCImage);
}