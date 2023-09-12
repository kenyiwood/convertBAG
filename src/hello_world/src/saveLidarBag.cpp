#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <memory>
#include <algorithm>
#include <signal.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <liblas/liblas.hpp>
#include "hello_world/saveLidarBag.hpp"
#include "hello_world/setProgressBar.hpp"

using namespace std;

const string LidarNodeTopic = "";
const string LidarNodeTopicL = "/lidar_fl/points_raw";
const string LidarNodeTopicR = "/lidar_fr/points_raw";
const string frameIDLidar = "";
const string frameIDLidarL = "velodyne_fl";
const string frameIDLidarR = "velodyne_fr";

static bool is_shutting_down = false;
const long int timeFrom1970ToNow = 1639872000;

class DataContainerBase
{
   public:
    DataContainerBase(
        const string &frame_id,
        const unsigned int init_width,
        const unsigned int init_height,
        const bool is_dense,
        int fields) :
        frame_id_(frame_id),
        init_width_(init_width),
        init_height_(init_height),
        is_dense_(is_dense)
    {
        // add fields
        cloud_.fields.clear();
        cloud_.fields.reserve(fields);

        int offset = 0;
        offset = addPointField(cloud_, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(cloud_, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(cloud_, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(cloud_, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(cloud_, "time", 1, sensor_msgs::PointField::FLOAT32, offset);

        cloud_.point_step = offset;
        cloud_.row_step = init_width_ * cloud_.point_step;
    }

    virtual void setup(const uint32_t &num_points, const ros::Time &stamp)
    {
        cloud_.header.stamp = stamp;
        cloud_.data.resize(num_points * cloud_.point_step);
        cloud_.width = init_width_;
        cloud_.height = init_height_;
        cloud_.is_dense = static_cast<uint8_t>(is_dense_);
    }

    virtual void addPoint(float x, float y, float z, const float intensity, const float time) = 0;

    const sensor_msgs::PointCloud2 &finishCloud()
    {
        cloud_.data.resize(cloud_.point_step * cloud_.width * cloud_.height);
        cloud_.row_step = cloud_.point_step * cloud_.width;
        cloud_.header.frame_id = frame_id_;

        return cloud_;
    }

   protected:
    string frame_id_;
    unsigned int init_width_;
    unsigned int init_height_;
    bool is_dense_;

    sensor_msgs::PointCloud2 cloud_;
};

class PointCloudXYZIT : public DataContainerBase
{
   public:
    PointCloudXYZIT(const string &frame_id = frameIDLidar) :
        DataContainerBase(frame_id, 0, 1, true, 5),
        iter_x_(cloud_, "x"),
        iter_y_(cloud_, "y"),
        iter_z_(cloud_, "z"),
        iter_intensity_(cloud_, "intensity"),
        iter_time_(cloud_, "time")
    {
    }

    void setup(const uint32_t &num_points, const ros::Time &stamp)
    {
        DataContainerBase::setup(num_points, stamp);
        iter_x_ = sensor_msgs::PointCloud2Iterator<float>(cloud_, "x");
        iter_y_ = sensor_msgs::PointCloud2Iterator<float>(cloud_, "y");
        iter_z_ = sensor_msgs::PointCloud2Iterator<float>(cloud_, "z");
        iter_intensity_ = sensor_msgs::PointCloud2Iterator<float>(cloud_, "intensity");
        iter_time_ = sensor_msgs::PointCloud2Iterator<float>(cloud_, "time");
    }

    void addPoint(float x, float y, float z, const float intensity, const float time)
    {
        *iter_x_ = x;
        *iter_y_ = y;
        *iter_z_ = z;
        *iter_intensity_ = intensity;
        *iter_time_ = time;

        ++cloud_.width;
        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
        ++iter_intensity_;
        ++iter_time_;
    }

   private:
    sensor_msgs::PointCloud2Iterator<float> iter_x_, iter_y_, iter_z_, iter_intensity_;
    sensor_msgs::PointCloud2Iterator<float> iter_time_;
};

void sigint_callback(int sig)
{
    ROS_INFO("shutting down!");
    is_shutting_down = true;
}

void readLASFile(string file, bool showDataDetail, string saveFileName, double &eachStartTime, double &eachEndTime, string LeftOrRight, int secondsBetweenIEAndROS, double &firstRecordTime)
{
    bool ifFirstRecordTime = false;
    /*
    if (is_shutting_down)
        ROS_INFO("shutting down!");*/

    int vectorCount = 0;
    double timeMarkStart = 0.0, timeMarkNow = 0.0;

    // pcl::FileReader(file);
    vector<vector<LASContent>> lascontent; // all las datas are saved as ros bag messages
    // double gpstime,x,y,z,intensity,colorRed,colorGreen,colorBlue;
    rosbag::Bag bag;

    unsigned long int pointsCount;
    ifstream ifs;
    ifs.open(file, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
        std::cout << file << "檔案開啟失敗!\n";
    else
    {
        liblas::ReaderFactory readerfactory;
        liblas::Reader reader = readerfactory.CreateWithStream(ifs);

        liblas::Header const &header = reader.GetHeader();
        pointsCount = header.GetPointRecordsCount();

        int counter = 0;

        vector<LASContent> lascontentTemp; // each point data vector

        // 進度列
        std::cout << boost::filesystem::path(file).string() + "點雲處理\n";
        ProgressBar *bar2 = new ProgressBar(pointsCount, "讀取點雲:");
        bar2->SetFrequencyUpdate(100);
        bar2->SetStyle("|", "-");

        double time, x, y, z, intensity;

        while (reader.ReadNextPoint())
        {
            liblas::Point const &point = reader.GetPoint();
            time = point.GetTime();
            x = point.GetX();
            y = point.GetY();
            z = point.GetZ();
            intensity = point.GetIntensity();

            double timeAppend = time + secondsBetweenIEAndROS;

            // 紀錄首筆時間 之後每筆資料皆扣掉首比時間
            // 2022.12.08 成大要求點雲時間改為記錄相對時間
            if (ifFirstRecordTime == false)
            {
                firstRecordTime = timeAppend;
                ifFirstRecordTime = true;
            }

            if (LeftOrRight == "Left")
            {
                if (counter == 0)
                {
                    eachStartTime = timeAppend;
                    timeMarkStart = timeAppend;
                }
                else
                {
                    eachEndTime = timeAppend;

                    if (timeAppend < eachStartTime)
                        eachStartTime = timeAppend;

                    if (timeAppend > eachEndTime)
                        eachEndTime = timeAppend;

                    timeMarkNow = timeAppend;
                }
            }
            else if (LeftOrRight == "Right")
            {
                if (counter == 0)
                    timeMarkStart = timeAppend;

                else
                    timeMarkNow = timeAppend;
            }
            LASContent oneContent;

            // ros timestamp
            timeAppend = timeAppend - firstRecordTime;  // 記錄相對時間 = 此筆資料時間 - 第一筆時間
            uint32_t sec = floor(timeAppend + firstRecordTime);  // stamp還是記錄真實時間
            uint32_t nsec = floor((timeAppend + firstRecordTime - sec) * 1000000000);
            ros::Time::init();
            ros::Time stamp(sec, nsec);
            oneContent.stamp = stamp;

            oneContent.gpstime = timeAppend;  // gpstime記錄真實時間
            oneContent.x = x;
            oneContent.y = y;
            oneContent.z = z;

            oneContent.intensity = intensity;

            if (LeftOrRight == "Left" && timeAppend > 0)
                lascontentTemp.push_back(oneContent);

            else if (LeftOrRight == "Right" && timeAppend > 0)
            {
                if (timeAppend >= (eachStartTime - firstRecordTime) && timeAppend <= (eachEndTime - firstRecordTime))
                    lascontentTemp.push_back(oneContent);
            }

            counter++;
            bar2->Progressed(counter);

            // 每1個frame記錄成一個message, 而1個frame約是0.100100秒, 裡面包含數十萬筆點雲資料
            if ((timeMarkNow - timeMarkStart) >= 0.100100 && lascontentTemp.size() > 0)
            {
                lascontent.push_back(lascontentTemp);
                vectorCount++;
                ifFirstRecordTime = false;
                lascontentTemp.clear();
                timeMarkStart = timeMarkNow;
            }
        }
        if (lascontentTemp.size() > 0)
            lascontent.push_back(lascontentTemp);
    }
    ifs.close();

    std::cout << "\n";

    signal(SIGINT, sigint_callback);

    ProgressBar *bar3 = setupProgressBarPartial("點雲轉檔:", lascontent.size());

    if (LeftOrRight == "Left")
        bag.open(saveFileName, rosbag::bagmode::Write);
    else if (LeftOrRight == "Right")
        bag.open(saveFileName, rosbag::bagmode::Append);

    for (int vector = 0; vector < lascontent.size(); vector++)
    {
        string frameIDLidar = frameIDLidarL;
        string LidarNodeTopic = LidarNodeTopicL;
        if (LeftOrRight == "Right")
        {
            frameIDLidar = frameIDLidarR;
            LidarNodeTopic = LidarNodeTopicR;
        }
        shared_ptr<PointCloudXYZIT> container(new PointCloudXYZIT(frameIDLidar));
        container->setup(lascontent[vector].size(), lascontent[vector][0].stamp);
        // cout << vector << endl;

        for (int i = 0; i < lascontent[vector].size(); i++)
        {
            container->addPoint(lascontent[vector][i].x,
                lascontent[vector][i].y,
                lascontent[vector][i].z,
                lascontent[vector][i].intensity,
                lascontent[vector][i].gpstime);
        }

        bag.write(LidarNodeTopic, lascontent[vector][0].stamp, container->finishCloud());
        // bag.close();

        bar3->Progressed(vector + 1);
    }

    std::cout << "\n 檔案輸出:" << saveFileName << "\n\n";

    if (showDataDetail)
    {
        for (int i = 0; i < lascontent.size(); i++)
        {
            for (int j = 0; j < lascontent[i].size(); j++)
                std::cout << i << ":" << lascontent[i][j].gpstime << "  "
                          << lascontent[i][j].x << "  "
                          << lascontent[i][j].y << "  "
                          << lascontent[i][j].z << "  "
                          << lascontent[i][j].intensity << "  "
                          << lascontent[i][j].colorRed << "  "
                          << lascontent[i][j].colorGreen << "  "
                          << lascontent[i][j].colorBlue << endl;
        }
    }
    ifFirstRecordTime = false;
}
