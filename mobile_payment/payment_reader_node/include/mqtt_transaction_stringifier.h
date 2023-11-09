#pragma once

#include <string>

#include "mobile_payment_interface/WorldpayTransResponse.h"
#include "mss_utils/json.hpp"


class MqttTransactionStringifier
{
    public:
        MqttTransactionStringifier(
                       mobile_payment_interface::WorldpayTransResponse response,
                       std::string container_sku,
                       std::string payment_reader_serial_num);

        std::string
        GetMqttTransactionString(void);

    private:
        const char* kTransIdJsonKey = "id";

        const char* kContainerSkuJsonKey = "sku";

        const char* kTransAmountJsonKey = "amount";

        const char* kReaderSerialNumJsonKey = "serial";

        const char* kResponseCodeJsonKey = "status";

        const char* kResponseCodeMessageJsonKey = "detail";

        const char* kIsMockResponseJsonKey = "is_mock";

        const char* kReferenceNumJsonKey = "reference_num";

        const char* kTransDateJsonKey = "date";

        const char* kTransTimeJsonKey = "time";

        const char* kTransTimeZoneJsonKey = "time_zone";

        const char* kApprovalNumJsonKey = "approval_num";

        nlohmann::json mqtt_transaction_json_;
};
