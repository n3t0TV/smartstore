#pragma once

#include <exception>
#include <string>


class AsyncCmdRunnerException : public std::exception
{
    public:
        AsyncCmdRunnerException();

        AsyncCmdRunnerException(std::string error_msg);

        const char* what() const noexcept override;

    private:
        std::string error_msg_;
};
