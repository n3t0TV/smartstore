#pragma once

#include <exception>
#include <string>


class ReaderDeviceException : public std::exception
{
    public:
        ReaderDeviceException(std::string error_msg);

        const char* what() const noexcept override;

    private:
        std::string error_msg_;
};
