#include <cstring>

#include "idtech/libIDT_Device.h"
#include "worldpay_utils.h"


std::string
mss_worldpay::DeviceErrCodeToString(int dev_err)
{
    static char device_msg[_CMD_BUF_LEN];

    memset(device_msg, 0, sizeof(device_msg));

    device_getIDGStatusCodeString(dev_err, device_msg);

    return device_msg;
}


std::string
mss_worldpay::DeviceResponseCodeToString(int dev_code)
{
    static char device_msg[_CMD_BUF_LEN];

    memset(device_msg, 0, sizeof(device_msg));

    device_getResponseCodeString(dev_code, device_msg);

    return device_msg;
}
