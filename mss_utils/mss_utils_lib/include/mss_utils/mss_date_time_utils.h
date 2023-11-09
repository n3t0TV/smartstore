#pragma once

#include <string>


namespace mss_date_time_utils {
    extern std::string
    GetDateInYyyymmddFormat(void);

    extern std::string
    GetTimeInHhmmssFormat(void);

    extern std::string
    GetTimeZone(void);
}
