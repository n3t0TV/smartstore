#include <mutex>
#include <ros/ros.h>

#include "mobile_payment_interface/WorldpayAccount.h"
#include "mss_utils/mss_ros_utils.h"
#include "mss_utils/persistence_reader_writer.h"
#include "worldpay_account.h"


/* Keys used to retrieve the persistent Worldpay account data */
static const char* kAccountIdKey = "account_id";
static const char* kAccountTokenKey = "account_token";
static const char* kAcceptorIdKey = "acceptor_id";

static mobile_payment_interface::WorldpayAccount worldpay_account;
static std::recursive_mutex worldpay_account_mutex;


void
mss_worldpay::UpdateAccount(mobile_payment_interface::WorldpayAccount account)
{
    auto& persistence = PersistenceReaderWriter::GetInstance();
    std::scoped_lock worldpay_account_lock(worldpay_account_mutex);

    persistence.WriteString(kAccountIdKey, account.id);
    persistence.WriteString(kAccountTokenKey, account.token);
    persistence.WriteString(kAcceptorIdKey, account.acceptor_id);
    persistence.Dump();

    worldpay_account = account;
}


mobile_payment_interface::WorldpayAccount
mss_worldpay::GetAccount(void)
{
    std::scoped_lock worldpay_account_lock(worldpay_account_mutex);
    static bool account_previously_read = false;

    try
    {
        if(!account_previously_read)
        {
            auto& persist = PersistenceReaderWriter::GetInstance();

            worldpay_account.id = persist.ReadString(kAccountIdKey);
            worldpay_account.token = persist.ReadString(kAccountTokenKey);
            worldpay_account.acceptor_id = persist.ReadString(kAcceptorIdKey);
            account_previously_read = true;
        }
    }
    catch(const std::invalid_argument& err)
    {
        worldpay_account.id = worldpay_account.kTortoiseAccountId;
        worldpay_account.token = worldpay_account.kTortoiseAccountToken;
        worldpay_account.acceptor_id = worldpay_account.kTortoiseAcceptorId;

        UpdateAccount(worldpay_account);

        MSS_ROS_WARN("Setting default values for Worldpay Account (id %s, token"
                     " %s, acceptor ID %s): %s", worldpay_account.id.c_str(),
                     worldpay_account.token.c_str(),
                     worldpay_account.acceptor_id.c_str(), err.what());
    }

    return worldpay_account;
}
