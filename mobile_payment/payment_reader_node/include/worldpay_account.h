#pragma once

#include <string>

#include "mobile_payment_interface/WorldpayAccount.h"


namespace mss_worldpay {
    void UpdateAccount(mobile_payment_interface::WorldpayAccount account);

    mobile_payment_interface::WorldpayAccount GetAccount(void);
}
