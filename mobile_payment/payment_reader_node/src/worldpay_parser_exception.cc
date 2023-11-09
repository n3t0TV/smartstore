#include "worldpay_parser_exception.h"


WorldpayParserException::WorldpayParserException(std::string error_msg):
    error_msg_(error_msg)
{
}


const char*
WorldpayParserException::what(void) const noexcept
{
    return error_msg_.c_str();
}
