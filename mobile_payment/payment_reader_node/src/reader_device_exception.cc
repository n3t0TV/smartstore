#include "reader_device_exception.h"


ReaderDeviceException::ReaderDeviceException(std::string error_msg):
    error_msg_(error_msg)
{
}


const char*
ReaderDeviceException::what(void) const noexcept
{
    return error_msg_.c_str();
}
