#include <fstream>
#include <sstream>

#include "mss_file_system_utils.h"
#include "mss_ros_utils.h"
#include "persistence_reader_writer.h"


PersistenceReaderWriter&
PersistenceReaderWriter::GetInstance(void)
{
    static PersistenceReaderWriter instance;

    return instance;
}


int
PersistenceReaderWriter::ReadInt(std::string key)
{
    int out_int = 0;
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    if(persitence_json_.contains(key)
       && ((persitence_json_[key].type()
            == nlohmann::json::value_t::number_integer)
           || (persitence_json_[key].type()
               == nlohmann::json::value_t::number_unsigned)))
    {
        out_int = persitence_json_[key].get<int>();
    }
    else
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The key \"" << key << "\" for int value couldn't be"
                       << " found in the persistent data.";

        throw std::invalid_argument(err_msg_stream.str());
    }

    return out_int;
}


void
PersistenceReaderWriter::WriteInt(std::string key, int value)
{
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    persitence_json_[key] = value;
}


std::string
PersistenceReaderWriter::ReadString(std::string key)
{
    std::string out_str;
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    if(persitence_json_.contains(key)
       && (persitence_json_[key].type() == nlohmann::json::value_t::string))
    {
        out_str = persitence_json_[key].get<std::string>();
    }
    else
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The key \"" << key << "\" for string value couldn't"
                       << " be found in the persistent data.";

        throw std::invalid_argument(err_msg_stream.str());
    }

    return out_str;
}


void
PersistenceReaderWriter::WriteString(std::string key, std::string value)
{
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    persitence_json_[key] = value;
}


float
PersistenceReaderWriter::ReadFloat(std::string key)
{
    float out_float;
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    if(persitence_json_.contains(key)
       && (persitence_json_[key].type()
           == nlohmann::json::value_t::number_float))
    {
        out_float = persitence_json_[key].get<float>();
    }
    else
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The key \"" << key << "\" for float value couldn't"
                       << " be found in the persistent data.";

        throw std::invalid_argument(err_msg_stream.str());
    }

    return out_float;
}


void
PersistenceReaderWriter::WriteFloat(std::string key, float value)
{
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    persitence_json_[key] = value;
}


bool
PersistenceReaderWriter::ReadBool(std::string key)
{
    bool out_bool;
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    if(persitence_json_.contains(key)
       && (persitence_json_[key].type()
           == nlohmann::json::value_t::boolean))
    {
        out_bool = persitence_json_[key].get<bool>();
    }
    else
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The key \"" << key << "\" for boolean value couldn't"
                       << " be found in the persistent data.";

        throw std::invalid_argument(err_msg_stream.str());
    }

    return out_bool;
}


void
PersistenceReaderWriter::WriteBool(std::string key, bool value)
{
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    persitence_json_[key] = value;
}


void
PersistenceReaderWriter::Dump(void)
{
    std::string file_path = GetPersistentFilePath();
    std::ofstream file_out_stream(file_path);
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    file_out_stream << persitence_json_ << std::endl;
}


PersistenceReaderWriter::PersistenceReaderWriter(void)
{
    CreatePersistenceFolderIfNotPresent();
    CreatePersistenceFileIfNotPresentOrInvalid();
    FillJsonFromFile();
}


void
PersistenceReaderWriter::CreatePersistenceFolderIfNotPresent(void)
{
    std::string folder_path = GetPersistentFolderPath();

    if(!mss_fs_utils::FolderExists(folder_path))
    {
        mss_fs_utils::CreateFolder(folder_path);
    }
}


void
PersistenceReaderWriter::CreatePersistenceFileIfNotPresentOrInvalid(void)
{
    std::string file_path = GetPersistentFilePath();

    if(!mss_fs_utils::FileExists(file_path) || !JsonFileValid(file_path))
    {
        CreateEmptyJsonFile(file_path);
    }
}


void
PersistenceReaderWriter::FillJsonFromFile(void)
{
    std::string json_file_path = GetPersistentFilePath();
    std::ifstream file_stream(json_file_path);
    std::scoped_lock persistence_json_lock_(persistence_json_mutex_);

    persitence_json_ = nlohmann::json::parse(file_stream);
}


std::string
PersistenceReaderWriter::GetPersistentFolderPath(void)
{
    std::stringstream folder_path_stream;

    folder_path_stream << mss_fs_utils::GetHomePath() << "/"
                       << kPersistentDataFolder << "/"
                       << mss_ros_utils::GetPackageName();

    return folder_path_stream.str();
}


std::string
PersistenceReaderWriter::GetPersistentFilePath(void)
{
    std::stringstream file_path_stream;

    file_path_stream << GetPersistentFolderPath() << "/"
                     << mss_ros_utils::GetNodeName() << kJsonExtension;

    return file_path_stream.str();
}


bool
PersistenceReaderWriter::JsonFileValid(std::string json_file_path)
{
    std::ifstream file_stream(json_file_path);

    return nlohmann::json::accept(file_stream);
}


void
PersistenceReaderWriter::CreateEmptyJsonFile(std::string file_path)
{
    std::stringstream cmd_stream;
    int error_code = 0;

    cmd_stream << kEmptyJsonCmd << " " << file_path;

    error_code = system(cmd_stream.str().c_str());
    if(error_code != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The command \"" << cmd_stream.str() << "\" failed"
                       << " while trying to create a file.";

        throw std::runtime_error(err_msg_stream.str());
    }
}
