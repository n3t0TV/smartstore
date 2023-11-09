#include <mutex>
#include <ros/ros.h>

#include "mss_utils/mss_ros_utils.h"
#include "mss_utils/persistence_reader_writer.h"
#include "worldpay_test_mode.h"


static const bool kDefaultTestMode = false;

static const char* kTestModeKey = "test_mode";

static std::recursive_mutex test_mode_mutex;
static bool worldpay_test_mode = true;


void
mss_worldpay::UpdateTestMode(bool test_mode)
{
    auto& persist = PersistenceReaderWriter::GetInstance();
    std::scoped_lock test_mode_lock(test_mode_mutex);

    persist.WriteBool(kTestModeKey, test_mode);
    persist.Dump();

    worldpay_test_mode = test_mode;
}


bool
mss_worldpay::GetTestMode(void)
{
    std::scoped_lock test_mode_lock(test_mode_mutex);
    static bool test_mode_previously_read = false;

    try
    {
        if(!test_mode_previously_read)
        {
            auto& persist = PersistenceReaderWriter::GetInstance();
            worldpay_test_mode = persist.ReadBool(kTestModeKey);
            test_mode_previously_read = true;
        }
    }
    catch(const std::invalid_argument& err)
    {
        UpdateTestMode(kDefaultTestMode);
        MSS_ROS_WARN("Setting default value for WorldPay test mode (%s): %s",
                     (kDefaultTestMode ? "enabled" : "disabled") ,err.what());
    }

    return worldpay_test_mode;
}
