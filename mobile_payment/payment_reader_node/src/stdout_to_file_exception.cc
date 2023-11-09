#include "stdout_to_file_exception.h"


StdoutToFileException::StdoutToFileException(std::string error_msg):
    error_msg_(error_msg)
{
}


const char*
StdoutToFileException::what(void) const noexcept
{
    return error_msg_.c_str();
}
