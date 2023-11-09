#include "async_cmd_runner_exception.h"


AsyncCmdRunnerException::AsyncCmdRunnerException()
{
}


AsyncCmdRunnerException::AsyncCmdRunnerException(std::string error_msg):
    error_msg_(error_msg)
{
}


const char*
AsyncCmdRunnerException::what(void) const noexcept
{
    return error_msg_.c_str();
}
