//
// Created by xc on 2021/4/9.
//

#ifndef ALOAM_VELODYNE_FILE_MANGER_H
#define ALOAM_VELODYNE_FILE_MANGER_H
// boost
#include <boost/filesystem.hpp>
//cpp standard
#include <string>
#include <iostream>
#include <fstream>

namespace LIRO{
    class FileManager{
    public:
        FileManager();
        ~FileManager();
        bool CreatDirectory(std::string dir_path);
        bool CreatFile(std::ofstream& ofs, std::string file_path);
        bool IsExist(std::string file_path);

    };
}
#endif //ALOAM_VELODYNE_FILE_MANGER_H
