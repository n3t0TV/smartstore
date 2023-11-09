#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

#include "mss_file_system_utils.h"


std::string
mss_fs_utils::GetHomePath(void)
{
    const char* kHomeEnvVar = "HOME";
    return getenv(kHomeEnvVar);
}


void
mss_fs_utils::CreateFolder(std::string folder_path)
{
    const char* kMkdirCmd = "mkdir -p";

    std::stringstream cmd_stream;
    int error_code = 0;

    cmd_stream << kMkdirCmd << " " << folder_path;

    error_code = system(cmd_stream.str().c_str());
    if(error_code != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The command \"" << cmd_stream.str() << "\" failed"
                       << " while trying to create a folder.";

        throw std::runtime_error(err_msg_stream.str());
    }
}


bool
mss_fs_utils::FileExists(std::string file_path)
{
    struct stat file_stat;
    bool exists = false;

    if(stat(file_path.c_str(), &file_stat) == 0)
    {
        /* If the file is a REGular file */
        if(file_stat.st_mode & S_IFREG)
        {
            exists = true;
        }
    }

    return exists;
}


bool
mss_fs_utils::FolderExists(std::string folder_path)
{
    struct stat folder_stat;
    bool exists = false;

    if(stat(folder_path.c_str(), &folder_stat) == 0)
    {
        if(folder_stat.st_mode & S_IFDIR)
        {
            exists = true;
        }
    }

    return exists;
}
