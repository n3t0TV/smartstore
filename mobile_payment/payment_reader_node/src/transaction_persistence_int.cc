#include <ros/ros.h>

#include "mss_utils/mss_ros_utils.h"
#include "mss_utils/persistence_reader_writer.h"
#include "transaction_persistence_int.h"


TransactionPersistenceInt::TransactionPersistenceInt(int min_val,
                                                     int max_val,
                                                     int default_val,
                                                     std::string key):
    kMinVal(min_val), kMaxVal(max_val), kDefaultVal(default_val),
    kPersistenceIntKey(key)
{
}


void
TransactionPersistenceInt::UpdateInt(int val)
{
    auto& persistence = PersistenceReaderWriter::GetInstance();
    std::scoped_lock timeout_lock(int_mutex_);
    int valid_val = GetValidInt(val);

    if(valid_val != current_val_)
    {
        persistence.WriteInt(kPersistenceIntKey.c_str(), valid_val);
        persistence.Dump();
        current_val_ = valid_val;
    }
}


int
TransactionPersistenceInt::GetInt(void)
{
    std::scoped_lock timeout_lock(int_mutex_);

    try
    {
        if(!int_read_)
        {
            auto& persist = PersistenceReaderWriter::GetInstance();

            current_val_ = persist.ReadInt(kPersistenceIntKey.c_str());
            UpdateInt(current_val_);
            int_read_ = true;
        }
    }
    catch(const std::invalid_argument& err)
    {
        UpdateInt(kDefaultVal);

        MSS_ROS_WARN("Setting default value for %s (%d): %s",
                     kPersistenceIntKey.c_str(), kDefaultVal, err.what());
    }

    return current_val_;
}


int
TransactionPersistenceInt::GetValidInt(int val)
{
    int valid_int = val;

    if(val < kMinVal)
    {
        valid_int = kMinVal;
    }

    if(val > kMaxVal)
    {
        valid_int = kMaxVal;
    }

    return valid_int;
}
