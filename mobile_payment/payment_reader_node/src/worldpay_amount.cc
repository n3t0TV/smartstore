#include <mutex>
#include <ros/ros.h>

#include "mss_utils/mss_ros_utils.h"
#include "mss_utils/persistence_reader_writer.h"
#include "worldpay_amount.h"


static const float kDefaultTransactionAmount = 1.0;

/* Key used to retrieve the persistent Worldpay amount data */
static const char* kTransactionAmountKey = "transaction_amount";

static float transaction_amount;
static std::recursive_mutex transaction_amount_mutex;


void
mss_worldpay::UpdateTransactionAmount(float amount)
{
    auto& persistence = PersistenceReaderWriter::GetInstance();
    std::scoped_lock amount_lock(transaction_amount_mutex);

    persistence.WriteFloat(kTransactionAmountKey, amount);
    persistence.Dump();

    transaction_amount = amount;
}


float
mss_worldpay::GetTransactionAmount(void)
{
    std::scoped_lock amount_lock(transaction_amount_mutex);
    static bool amount_previously_read = false;

    try
    {
        if(!amount_previously_read)
        {
            auto& persist = PersistenceReaderWriter::GetInstance();

            transaction_amount = persist.ReadFloat(kTransactionAmountKey);
            amount_previously_read = true;
        }
    }
    catch(const std::invalid_argument& err)
    {
        UpdateTransactionAmount(kDefaultTransactionAmount);

        MSS_ROS_WARN("Setting default value for transaction amount (%.2f): %s",
                     kDefaultTransactionAmount, err.what());
    }

    return transaction_amount;
}
