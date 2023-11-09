#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <sstream>

#include "stdout_to_file.h"
#include "stdout_to_file_exception.h"


StdoutToFile::StdoutToFile(void)
{
    try
    {
        file_path_ = (std::filesystem::temp_directory_path().string()
                      + kTempFileName);

        BackupStdout();
        CreateTempFile();
        ReplaceStdoutWithTempFile();
        CloseTempFileDescriptor();
    }
    catch(const StdoutToFileException& err)
    {
        /* Attempt to restore STDOUT */
        if(backup_stdout_fd_ == kInvalidFd)
        {
            (void) dup2(backup_stdout_fd_, STDOUT_FILENO);
            (void) close(backup_stdout_fd_);
        }

        /* Attempt to close temp file */
        if(temp_file_fd_ == kInvalidFd)
        {
            (void) close(temp_file_fd_);
        }
        throw;
    }
}


StdoutToFile::~StdoutToFile(void)
{
    RestoreStdout();
    PrintFileToStdout();
}


std::string
StdoutToFile::GetFilePath(void)
{
    return file_path_;
}


void
StdoutToFile::BackupStdout(void)
{
    backup_stdout_fd_ = dup(STDOUT_FILENO);

    if(backup_stdout_fd_ == kInvalidFd)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't backup STDOUT in another file descriptor: "
                       << std::strerror(errno);

        throw StdoutToFileException(err_msg_stream.str());
    }
}


void
StdoutToFile::CreateTempFile(void)
{
    temp_file_fd_ = creat(file_path_.c_str(),
                          (mode_t) (S_IRUSR | S_IWUSR | S_IRGRP));

    if(temp_file_fd_ == kInvalidFd)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't create temporary file to replace STDOUT: "
                       << std::strerror(errno);

        throw StdoutToFileException(err_msg_stream.str());
    }
}


void
StdoutToFile::ReplaceStdoutWithTempFile(void)
{
    if(dup2(temp_file_fd_, STDOUT_FILENO) == -1)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't replace STDOUT with temporary file: "
                       << std::strerror(errno);

        throw StdoutToFileException(err_msg_stream.str());
    }
}


void
StdoutToFile::CloseTempFileDescriptor(void)
{
    if(close(temp_file_fd_) == -1)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't close original temporary file descriptor: "
                       << std::strerror(errno);

        throw StdoutToFileException(err_msg_stream.str());
    }
}


void
StdoutToFile::RestoreStdout(void)
{
    ReplaceTempFileWithBackupStdout();
    CloseBackupStdout();
}


void
StdoutToFile::PrintFileToStdout(void)
{
    std::ifstream file_stream = std::ifstream(file_path_);
    std::string stream_line;

    while(file_stream)
    {
        std::getline(file_stream, stream_line);
        if(!stream_line.empty())
        {
            (void) printf("%s\n", stream_line.c_str());
        }
    }
}


void
StdoutToFile::ReplaceTempFileWithBackupStdout(void)
{
    if(dup2(backup_stdout_fd_, STDOUT_FILENO) == -1)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't restore STDOUT: " << std::strerror(errno);

        throw StdoutToFileException(err_msg_stream.str());
    }
}


void
StdoutToFile::CloseBackupStdout(void)
{
    if(close(backup_stdout_fd_) == -1)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't close backup STDOUT file descriptor: "
                       << std::strerror(errno);

        throw StdoutToFileException(err_msg_stream.str());
    }
}
