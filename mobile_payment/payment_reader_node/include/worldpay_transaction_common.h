#pragma once

#include <string>

#include "mobile_payment_interface/WorldpayAccount.h"
#include "worldpay_account.h"


namespace mss_worldpay {
    typedef struct
    {
        mobile_payment_interface::WorldpayAccount account;
        std::string reference_number;
        int ticket_number;
        std::string terminal_id;
        bool is_test;
    }
    TransactionCommon;
}
