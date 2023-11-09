#pragma once

#include <string>


namespace mss_fs_utils {
    extern std::string
    GetHomePath(void);


    extern void
    CreateFolder(std::string folder_path);


    extern bool
    FileExists(std::string file_path);


    extern bool
    FolderExists(std::string folder_path);
}
