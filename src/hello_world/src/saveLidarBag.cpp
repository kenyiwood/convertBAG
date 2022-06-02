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
#include "hello_world/progress_bar.hpp"

using namespace std;

const string LidarNodeTopic = "/lidar_fl/points_raw";
const string frameIDLidar = "velodyne_fl";

static bool is_shutting_down = false;
const long int timeFrom1970ToNow = 1617408000;

class DataContainerBase
{
public:
    DataContainerBase(
        const string &frame_id,
        const unsigned int init_width,
        const unsigned int init_height,
        const bool is_dense,
        int fields) : frame_id_(frame_id),
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
    PointCloudXYZIT(const string &frame_id = frameIDLidar) : DataContainerBase(frame_id, 0, 1, true, 5),
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

void readLASFile(string file, bool showDataDetail, string saveFileName)
{
    if (is_shutting_down)
        ROS_INFO("shutting down!");

    int vectorCount = 0, vectorOneTotal = 10000;

    // pcl::FileReader(file);
    vector<vector<LASContent>> lascontent;
    // double gpstime,x,y,z,intensity,colorRed,colorGreen,colorBlue;
    rosbag::Bag bag;
    unsigned long int pointsCount;

    ifstream ifs;
    ifs.open(file, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
        cout << file << "檔案開啟失敗!\n";
    else
    {
        liblas::ReaderFactory readerfactory;
        liblas::Reader reader = readerfactory.CreateWithStream(ifs);

        liblas::Header const &header = reader.GetHeader();
        pointsCount = header.GetPointRecordsCount();

        int counter = 0;

        vector<LASContent> lascontentTemp;

        // 進度列
        cout << boost::filesystem::path(file).string() + "點雲處理\n";
        ProgressBar *bar2 = new ProgressBar(pointsCount, "讀取點雲:");
        bar2->SetFrequencyUpdate(100);
        bar2->SetStyle("|", "-");

        while (reader.ReadNextPoint())
        {
            liblas::Point const &p = reader.GetPoint();
            LASContent oneContent;

            // ros timestamp
            double time = p.GetTime();
            uint32_t sec = floor(time) + timeFrom1970ToNow;
            uint32_t nsec = floor((time - sec) * 1000000000);
            ros::Time::init();
            ros::Time stamp(sec, nsec);
            oneContent.stamp = stamp;

            oneContent.gpstime = p.GetTime();
            oneContent.x = p.GetX();
            oneContent.y = p.GetY();
            oneContent.z = p.GetZ();

            oneContent.intensity = p.GetIntensity();
            oneContent.colorRed = p.GetColor().GetRed();
            oneContent.colorGreen = p.GetColor().GetGreen();
            oneContent.colorBlue = p.GetColor().GetBlue();

            if (counter >= vectorOneTotal * (vectorCount + 1))
            {
                lascontent.push_back(lascontentTemp);
                vectorCount++;
                lascontentTemp.clear();
            }

            lascontentTemp.push_back(oneContent);
            counter++;

            bar2->Progressed(counter);
        }
        if (lascontentTemp.size() > 0)
            lascontent.push_back(lascontentTemp);
    }
    ifs.close();
    // cout << pointsCount << "  " << lascontent.size() << "  "
    //      << lascontent[lascontent.size() - 1].size() << endl;

    cout << "\n";

    signal(SIGINT, sigint_callback);

    ProgressBar *bar3 = new ProgressBar(lascontent.size(), "點雲轉檔:");
    bar3->SetFrequencyUpdate(100);
    bar3->SetStyle("|", "-");
    for (int vector = 0; vector < lascontent.size(); vector++)
    {
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
        if (vector == 0)
        {
            bag.open(saveFileName, rosbag::bagmode::Write);
        }
        else
            bag.open(saveFileName, rosbag::bagmode::Append);

        bag.write(LidarNodeTopic, lascontent[vector][0].stamp, container->finishCloud());
        bag.close();

        bar3->Progressed(vector + 1);
    }

    cout << "\n檔案輸出:" << saveFileName << "\n\n";

    if (showDataDetail)
    {
        for (int i = 0; i < lascontent.size(); i++)
        {
            for (int j = 0; j < lascontent[i].size(); j++)
                cout << i << ":" << lascontent[i][j].gpstime << "  "
                     << lascontent[i][j].x << "  "
                     << lascontent[i][j].y << "  "
                     << lascontent[i][j].z << "  "
                     << lascontent[i][j].intensity << "  "
                     << lascontent[i][j].colorRed << "  "
                     << lascontent[i][j].colorGreen << "  "
                     << lascontent[i][j].colorBlue << endl;
        }
    }
}

/*
void convertLASFileContentToBAG(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    // cout << "" << endl;
    // sensor_msgs::PointCloud2 output;
    // for (int i)

    // write sensor_msgs/PointCloud2
    uint32_t sec=111,nsec=222;
    ros::Time::init();
    ros::Time stamp(sec,nsec);
    rosbag::Bag bag(saveFileName,rosbag::bagmode::Write);
    sensor_msgs::PointCloud2 pointCloud;

    pointCloud.header.stamp=stamp;
    pointCloud.header.seq=0;
    pointCloud.header.frame_id=frameIDLidar;

    // 2D structure of the point cloud. If the cloud is unordered, height is
    // 1 and width is the length of the point cloud.
    int num_points=1,point_step=20;
    pointCloud.height=1;
    pointCloud.width=num_points; // num_points, width X point_step = row_step

    pointCloud.fields[0].name = "x";
    pointCloud.fields[0].offset=0;
    pointCloud.fields[0].datatype=7;
    pointCloud.fields[0].count=1;

    pointCloud.fields[1].name = "y";
    pointCloud.fields[1].offset=4;
    pointCloud.fields[1].datatype=7;
    pointCloud.fields[1].count=1;

    pointCloud.fields[2].name = "z";
    pointCloud.fields[2].offset=8;
    pointCloud.fields[2].datatype=7;
    pointCloud.fields[2].count=1;

    pointCloud.fields[3].name = "intensity";
    pointCloud.fields[3].offset=12;
    pointCloud.fields[3].datatype=7;
    pointCloud.fields[3].count=1;

    pointCloud.fields[4].name = "time";
    pointCloud.fields[4].offset=16;
    pointCloud.fields[4].datatype=7;
    pointCloud.fields[4].count=1;

    pointCloud.is_bigendian=false;
    pointCloubag.close();d.point_step=point_step;
    pointCloud.row_step=point_step*num_points;

}*/
