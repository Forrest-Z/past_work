//
// Created by xc on 2020/11/9.
//

#ifndef GPLIO_FILE_MANAGER_H
#define GPLIO_FILE_MANAGER_H

// boost
#include <boost/filesystem.hpp>
//cpp standard
#include <string>
#include <iostream>
#include <fstream>

namespace gp_lio{
    class FileManager{
    public:
        FileManager();
        ~FileManager();
        bool CreatDirectory(std::string dir_path);
        bool CreatFile(std::ofstream& ofs, std::string file_path);
        bool IsExist(std::string file_path);

    };
}



#endif //GPLIO_FILE_MANAGER_H
