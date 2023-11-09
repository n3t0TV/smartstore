#pragma once

#include <exception>
#include <string>


class WorldpayParserException : public std::exception
{
    public:
        WorldpayParserException(std::string error_msg);

        const char* what() const noexcept override;

    private:
        std::string error_msg_;
};
