#pragma once

#include <mutex>
#include <string>

#include "json.hpp"


class PersistenceReaderWriter
{
    public:
        static PersistenceReaderWriter& GetInstance(void);

        int ReadInt(std::string key);

        void WriteInt(std::string key, int value);

        std::string ReadString(std::string key);

        void WriteString(std::string key, std::string value);

        float ReadFloat(std::string key);

        void WriteFloat(std::string key, float value);

        bool ReadBool(std::string key);

        void WriteBool(std::string key, bool value);

        void Dump(void);

    private:
        const char* kPersistentDataFolder = ".mss_persistent_data";

        const char* kEmptyJsonCmd = "echo \"{}\" >";

        const char* kJsonExtension = ".json";

        nlohmann::json persitence_json_;

        std::mutex persistence_json_mutex_;

        PersistenceReaderWriter(void);

        /* Don't implement this constructor and keep it inaccessible to avoid
         * copies of the singleton */
        PersistenceReaderWriter(PersistenceReaderWriter const&);

        /* Don't implement this function and keep it inaccessible to avoid
         * copies of the singleton */
        void operator=(PersistenceReaderWriter const&);

        void CreatePersistenceFolderIfNotPresent(void);

        void CreatePersistenceFileIfNotPresentOrInvalid(void);

        void FillJsonFromFile(void);

        std::string GetPersistentFolderPath(void);

        std::string GetPersistentFilePath(void);

        bool JsonFileValid(std::string json_file_path);

        void CreateEmptyJsonFile(std::string file_path);
};
