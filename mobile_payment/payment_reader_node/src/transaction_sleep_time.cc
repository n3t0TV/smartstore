#include <string>

#include "transaction_persistence_int.h"
#include "transaction_sleep_time.h"


static const int kMaxTransSleepMilliSec = 60000;
static const int kMinTransSleepMilliSec = 500;
static const int kDefaultSleepMilliSec= 20000;

/* Key used to retrieve the persistent sleep time between transactions */
static const std::string kTransactionSleepTimeKey = "transaction_sleep_time";

TransactionPersistenceInt sleep_time_persistence(kMinTransSleepMilliSec,
                                                 kMaxTransSleepMilliSec,
                                                 kDefaultSleepMilliSec,
                                                 kTransactionSleepTimeKey);


void
mss_worldpay::UpdateTransactionSleepTime(int sleep_time_ms)
{
    sleep_time_persistence.UpdateInt(sleep_time_ms);
}


int
mss_worldpay::GetTransactionSleepTime(void)
{
    return sleep_time_persistence.GetInt();
}
