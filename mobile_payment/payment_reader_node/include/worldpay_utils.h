#pragma once

#include <string>


namespace mss_worldpay {
    std::string DeviceErrCodeToString(int dev_err);

    std::string DeviceResponseCodeToString(int dev_code);
}
