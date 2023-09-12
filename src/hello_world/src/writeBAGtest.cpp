#include <iostream>
#include <boost/filesystem.hpp>
#include <string>
#include <iomanip>
#include "hello_world/saveIMUBag.hpp"
#include "hello_world/saveImageBag.hpp"
#include "hello_world/saveLidarBag.hpp"
#include "hello_world/getAllFiles.hpp"
#include "hello_world/setProgressBar.hpp"
#include "hello_world/timeConverter.hpp"

using namespace std;

vector<string> lasfilesLeft;  // 左邊光達檔案列表
vector<string> lasfilesRight;  // 右邊光達檔案列表
vector<string> iefiles;  // ie解算軌跡檔案列表
vector<string> jpgfiles;  // 影像檔案列表
vector<IEContent> iecontent;  // ie解成果內容
vector<ImageContent> imagecontent;  // 影像時間內容

const string ieExtension = ".txt";
const string lasExtension = ".las";
const string jpgExtension = ".jpg";

const string lasSubDirectoryL = "/pointcloud/L";
const string lasSubDirectoryR = "/pointcloud/R";
const string ieSubDirectory = "/ie";
const string jpgSubDirectory = "/jpg";

double eachStartTime = 0.0, eachEndTime = 0.0;  // 區段時間
double firstRecordTime = 0.0; // 第一筆點雲時間
int dataYear = 0, dataMonth = 0, dataDay = 0, secondsBetweenIEAndROS = 0;
int progressBarValue = 1;

void showHeader(char** argv)
{
    // 讀軌跡解轉bag
    cout << "---BAG轉檔處理---" << endl;
    string inputDirectory = argv[1];
    cout << "資料存放路徑:" << argv[1] << endl;
    string outputDirectory = argv[2];
    cout << "BAG成果儲存路徑:" << argv[2] << endl;

    cout << "開始搜尋資料夾內檔案進行轉檔\n\n";
}

void readImageFiles(string& imageTimeFilePath, vector<string>& jpgfiles)
{
    for (int i = 0; i < jpgfiles.size(); i++)
    {
        string inputFile = jpgfiles[i];

        // 處理jpg檔
        if (jpgfiles[i].find(jpgExtension) != string::npos)
        {
            ImageContent imageTemp;
            imageTemp.imageFilePath = jpgfiles[i];
            imageTemp.filename = boost::filesystem::path(jpgfiles[i]).filename().c_str();
            imagecontent.push_back(imageTemp);
        }
        // 處理影像時間檔
        if (jpgfiles[i].find(ieExtension) != string::npos)
        {
            imageTimeFilePath = jpgfiles[i];
        }
    }
}

void readImageTimeFile(string imageTimeFilePath, vector<ImageContent>& imagecontent, int secondsBetweenIEAndROS)
{
    ifstream ifs;
    ifs.open(imageTimeFilePath, std::ios::in);
    if (!ifs.is_open())
        cout << imageTimeFilePath << "檔案開啟失敗!\n";
    else
    {
        string imageFileName;
        double time;
        while (ifs >> imageFileName >> time)
        {
            for (int i = 0; i < imagecontent.size(); i++)
            {
                if (imagecontent[i].filename == imageFileName)
                {
                    imagecontent[i].gpstime = time + secondsBetweenIEAndROS;
                    break;
                }
            }
        }
    }
    ifs.close();
}

void getDataTime(const char* filename, int& year, int& month, int& day)
{
    string inputFilenameYear = "", inputFilenameMonth = "", inputFilenameDay = "";
    for (int i = 0; i < 4; i++)
        inputFilenameYear += filename[i];
    for (int i = 4; i < 6; i++)
        inputFilenameMonth += filename[i];
    for (int i = 6; i < 8; i++)
        inputFilenameDay += filename[i];
    year = stoi(inputFilenameYear);
    month = stoi(inputFilenameMonth);
    day = stoi(inputFilenameDay);
}

void processDataConvertToBAG(vector<string> lasfiles, string lasDirectory, string outputDirectory,
    string LeftOrRight, ProgressBar* bar)
{
    for (int i = 0; i < lasfiles.size(); i++)
    {
        string inputFile = lasfiles[i];
        // 處理點雲las檔
        if (lasfiles[i].find(lasExtension) != string::npos)
        {
            string saveFileName = inputFile.replace(inputFile.find(lasDirectory),
                                               lasDirectory.size(), outputDirectory)
                                      .replace(inputFile.find(lasExtension), lasExtension.size(), "_2Lidar.bag");
            // 先處理左光達 再處理右光達
            readLASFile(lasfiles[i], false, saveFileName, eachStartTime, eachEndTime, LeftOrRight, secondsBetweenIEAndROS, firstRecordTime);
            readLASFile(lasfilesRight[i], false, saveFileName, eachStartTime, eachEndTime, "Right", secondsBetweenIEAndROS, firstRecordTime);
            readImageFile(imagecontent, saveFileName, eachStartTime, eachEndTime);
            convertIEFileContentToBAG(iecontent, saveFileName, eachStartTime, eachEndTime);
        }

        bar->Progressed(progressBarValue);
        progressBarValue++;
        cout << "\n\n";
    }
}

int main(int argc, char** argv)
{
    showHeader(argv);
    string inputDirectory = argv[1];
    string outputDirectory = argv[2];

    string oriInputDirectory0, oriInputDirectory1, oriInputDirectory2, oriInputDirectory3;
    oriInputDirectory0 = oriInputDirectory1 = oriInputDirectory2 = oriInputDirectory3 = inputDirectory;

    string lasDirectoryL = oriInputDirectory0.append(lasSubDirectoryL);
    string lasDirectoryR = oriInputDirectory1.append(lasSubDirectoryR);

    getAllFiles(lasDirectoryL, lasfilesLeft);  // 取得檔案列表
    getAllFiles(lasDirectoryR, lasfilesRight);  // 取得檔案列表

    string ieDirectory = oriInputDirectory2.append(ieSubDirectory);
    getAllFiles(ieDirectory, iefiles);  // 取得檔案列表

    string jpgDirectory = oriInputDirectory3.append(jpgSubDirectory);
    getAllFiles(jpgDirectory, jpgfiles);  // 取得檔案列表

    // 進度條
    int progressBarLength = lasfilesLeft.size();
    ProgressBar* bar = setupProgressBarMain("完成進度：", progressBarLength);

    // 讀取影像列表
    string imageTimeFilePath;
    readImageFiles(imageTimeFilePath, jpgfiles);

    // 讀取影像時間檔
    getDataTime(boost::filesystem::path(imageTimeFilePath).filename().c_str(), dataYear, dataMonth, dataDay);
    secondsBetweenIEAndROS = secondsDifferenceBetweenIEAndROS(dataYear, dataMonth, dataDay);
    readImageTimeFile(imageTimeFilePath, imagecontent, secondsBetweenIEAndROS);

    // read IE Content to iecontent and set time interval to savefiles
    readIEFile(iefiles[0], iecontent, secondsBetweenIEAndROS);

    // 針對資料夾內所有檔案進行轉檔 先處理左光達 再處理右光達
    processDataConvertToBAG(lasfilesLeft, lasDirectoryL, outputDirectory, "Left", bar);
    // processDataConvertToBAG(lasfilesRight, lasDirectoryR, outputDirectory, "Right", bar);

    cout << "\n程式執行完畢！\n";
    return 0;
}
