#include "card_tap_monitor_exception.h"


CardTapMonitorException::CardTapMonitorException()
{
}


CardTapMonitorException::CardTapMonitorException(std::string error_msg):
    error_msg_(error_msg)
{
}


const char*
CardTapMonitorException::what(void) const noexcept
{
    return error_msg_.c_str();
}
