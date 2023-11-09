#pragma once

#include <exception>
#include <string>


class StdoutToFileException : public std::exception
{
    public:
        StdoutToFileException(std::string error_msg);

        const char* what() const noexcept override;

    private:
        std::string error_msg_;
};
