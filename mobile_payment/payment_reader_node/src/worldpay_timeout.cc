#include <string>

#include "transaction_persistence_int.h"
#include "worldpay_timeout.h"


static const int kMaxTimeoutSec = 180;
static const int kMinTimeoutSec = 10;
static const int kDefaultTransactionTimeout = kMaxTimeoutSec;

/* Key used to retrieve the persistent Worldpay timeout data */
static const std::string kTransactionTimeoutKey = "transaction_timeout";


TransactionPersistenceInt timeout_persistence(kMinTimeoutSec, kMaxTimeoutSec,
                                              kDefaultTransactionTimeout,
                                              kTransactionTimeoutKey);


void
mss_worldpay::UpdateTransactionTimeout(int timeout)
{
    timeout_persistence.UpdateInt(timeout);
}


int
mss_worldpay::GetTransactionTimeout(void)
{
    return timeout_persistence.GetInt();
}
