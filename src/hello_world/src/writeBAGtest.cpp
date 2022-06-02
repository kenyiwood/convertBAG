#include <dirent.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include "hello_world/progress_bar.hpp"
#include "hello_world/saveIMUBag.hpp"
#include "hello_world/saveImageBag.hpp"
#include "hello_world/saveLidarBag.hpp"

using namespace std;

const string ieExtension = ".txt";
const string lasExtension = ".las";
const string jpgExtension = ".jpg";

void getAllFiles(string path, vector<string>& files)
{
    struct dirent* dirent;
    DIR* directory;
    directory = opendir(path.c_str());

    if (directory != NULL)
    {
        for (dirent = readdir(directory); dirent != NULL; dirent = readdir(directory))
        {
            if (dirent->d_type == DT_REG)  // if d is a regular file
            {
                string filePath = path + "/" + dirent->d_name;
                if (filePath.find(ieExtension) != string::npos || filePath.find(lasExtension) != string::npos || filePath.find(jpgExtension) != string::npos)
                    files.push_back(filePath);
                // cout << path << "/" << dirent->d_name << endl;
            }
            else if (dirent->d_type == DT_DIR)
            {
                //如果讀取的d_name為 . 或者..
                //表示讀取的是當前目錄符和上一目錄符,
                //用contiue跳過，不進行下面的輸出
                if (strcmp(dirent->d_name, ".") == 0 || strcmp(dirent->d_name, "..") == 0)
                    continue;
                else
                    getAllFiles(path + "/" + dirent->d_name, files);
            }
        }
    }
}

void duplicateDirectories(string inputPath, string outputPath)
{
    struct dirent* dirent;
    DIR* directory;
    directory = opendir(inputPath.c_str());

    if (directory != NULL)
    {
        for (dirent = readdir(directory); dirent != NULL; dirent = readdir(directory))
        {
            if (dirent->d_type == DT_DIR)
            {
                //如果讀取的d_name為 . 或者..
                //表示讀取的是當前目錄符和上一目錄符,
                //用contiue跳過，不進行下面的輸出
                if (strcmp(dirent->d_name, ".") == 0 || strcmp(dirent->d_name, "..") == 0)
                    continue;
                else
                {
                    string inputSubDirectory = inputPath + "/" + dirent->d_name;
                    string outputSubDirectory = outputPath + "/" + dirent->d_name;
                    boost::filesystem::create_directories(outputSubDirectory);
                    duplicateDirectories(inputSubDirectory, outputSubDirectory);
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    // 讀軌跡解轉bag
    cout << "---BAG轉檔處理---" << endl;
    string inputDirectory = argv[1];
    cout << "資料存放路徑:" << argv[1] << endl;
    string outputDirectory = argv[2];
    cout << "BAG成果儲存路徑:" << argv[2] << endl;

    vector<string> files;
    getAllFiles(inputDirectory, files);  // 取得檔案列表
    duplicateDirectories(inputDirectory, outputDirectory);  // 複製子資料夾到輸出目錄

    // 儲存影像路徑
    vector<ImageContent> imagecontent;

    cout << "開始搜尋資料夾內檔案進行轉檔\n\n";
    // 針對資料夾內所有檔案進行轉檔
    for (int i = 0; i < files.size(); i++)
    {
        string inputFile = files[i];

        // 處理jpg檔
        if (files[i].find(jpgExtension) != string::npos)
        {
            ImageContent imageTemp;
            imageTemp.imageFilePath = files[i];
            imagecontent.push_back(imageTemp);
            // string saveFileName =
            // inputFile.replace(inputFile.find(inputDirectory),
            // inputDirectory.size(), outputDirectory) + ".bag";
        }
    }
    readImageFile(imagecontent, outputDirectory);

    // 進度條
    int progressBarLength = files.size() - imagecontent.size();  // 光達與軌跡解數量＝全部-影像
    ProgressBar* bar = new ProgressBar(progressBarLength, "完成進度：");
    if (progressBarLength < 100)
        bar->SetFrequencyUpdate(progressBarLength);
    else
        bar->SetFrequencyUpdate(100);
    bar->SetStyle("\u2588", "-");

    int progressBarValue = 1;

    for (int i = 0; i < files.size(); i++)
    {
        string inputFile = files[i];

        // 處理txt檔
        if (files[i].find(ieExtension) != string::npos)
        {
            string saveFileName = inputFile.replace(inputFile.find(inputDirectory), inputDirectory.size(), outputDirectory) + ".bag";
            readIEFile(files[i], false, saveFileName);
        }

        // 處理las檔
        if (files[i].find(lasExtension) != string::npos)
        {
            string saveFileName = inputFile.replace(inputFile.find(inputDirectory), inputDirectory.size(), outputDirectory) + ".bag";
            readLASFile(files[i], false, saveFileName);
        }

        if (files[i].find(ieExtension) != string::npos || files[i].find(lasExtension) != string::npos)
        {
            bar->Progressed(progressBarValue);
            progressBarValue++;
            cout << endl;
        }
    }

    cout << "\n程式執行完畢！\n";
    return 0;
}
