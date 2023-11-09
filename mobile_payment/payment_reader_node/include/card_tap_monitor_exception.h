#pragma once

#include <exception>
#include <string>


class CardTapMonitorException : public std::exception
{
    public:
        CardTapMonitorException();

        CardTapMonitorException(std::string error_msg);

        const char* what() const noexcept override;

    private:
        std::string error_msg_;
};
