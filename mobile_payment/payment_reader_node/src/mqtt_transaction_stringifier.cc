#include "mqtt_transaction_stringifier.h"


MqttTransactionStringifier::MqttTransactionStringifier(
                       mobile_payment_interface::WorldpayTransResponse response,
                       std::string container_sku,
                       std::string payment_reader_serial_num)
{
    mqtt_transaction_json_[kResponseCodeJsonKey]
                                               = response.express_response_code;
    mqtt_transaction_json_[kResponseCodeMessageJsonKey]
                                            = response.express_response_message;
    mqtt_transaction_json_[kIsMockResponseJsonKey] = response.is_mock_response;
    mqtt_transaction_json_[kReferenceNumJsonKey] = response.reference_number;

    mqtt_transaction_json_[kContainerSkuJsonKey] = container_sku;
    mqtt_transaction_json_[kReaderSerialNumJsonKey] = payment_reader_serial_num;

    if(response.express_response_code == response.kApprovedResponseCode)
    {
        mqtt_transaction_json_[kTransIdJsonKey] = response.transaction_id;
        mqtt_transaction_json_[kApprovalNumJsonKey] = response.approval_number;
        mqtt_transaction_json_[kTransAmountJsonKey] = response.approved_amount;

        mqtt_transaction_json_[kTransDateJsonKey]
                                            = response.express_transaction_date;
        mqtt_transaction_json_[kTransTimeJsonKey]
                                            = response.express_transaction_time;
        mqtt_transaction_json_[kTransTimeZoneJsonKey]
                                        = response.express_transaction_timezone;
    }
}


std::string
MqttTransactionStringifier::GetMqttTransactionString(void)
{
    return mqtt_transaction_json_.dump();
}
