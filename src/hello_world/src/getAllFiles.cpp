#include <iostream>
#include <dirent.h>
#include <string.h>
#include <vector>

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