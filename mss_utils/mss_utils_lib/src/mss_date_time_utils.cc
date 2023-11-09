#include <chrono>
#include <iomanip>
#include <sstream>

#include "mss_date_time_utils.h"


static std::string
GetTimeDateStringFromFormat(const char* format);


std::string
mss_date_time_utils::GetDateInYyyymmddFormat(void)
{
    const char* kDateFormatString = "%Y%m%d";
    return GetTimeDateStringFromFormat(kDateFormatString);
}


std::string
mss_date_time_utils::GetTimeInHhmmssFormat(void)
{
    const char* kTimeFormatString = "%H%M%S";
    return GetTimeDateStringFromFormat(kTimeFormatString);
}


std::string
mss_date_time_utils::GetTimeZone(void)
{
    const char* kTimeZoneFormatString = "%z";
    std::stringstream time_zone_stream;

    time_zone_stream << "UTC"
                     << GetTimeDateStringFromFormat(kTimeZoneFormatString);
    return time_zone_stream.str();
}


std::string
GetTimeDateStringFromFormat(const char* format)
{
    std::stringstream time_date_stream;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    time_date_stream << std::put_time(std::localtime(&in_time_t), format);

    return time_date_stream.str();
}
