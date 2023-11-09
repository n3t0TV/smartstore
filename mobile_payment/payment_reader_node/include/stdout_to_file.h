#pragma once

#include <string>


class StdoutToFile
{
    public:
        StdoutToFile(void);

        ~StdoutToFile(void);

        std::string GetFilePath(void);

    private:
        const char* kTempFileName = "/temp_payment_reader.log";

        const int kInvalidFd = -1;

        int backup_stdout_fd_ = kInvalidFd;

        int temp_file_fd_ = kInvalidFd;

        std::string file_path_;

        void BackupStdout(void);

        void CreateTempFile(void);

        void ReplaceStdoutWithTempFile(void);

        void CloseTempFileDescriptor(void);

        void RestoreStdout(void);

        void PrintFileToStdout(void);

        void ReplaceTempFileWithBackupStdout(void);

        void CloseBackupStdout(void);
};
