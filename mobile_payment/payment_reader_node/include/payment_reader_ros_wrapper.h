#pragma once

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include "mobile_payment_interface/WorldpayAccount.h"
#include "mobile_payment_interface/WorldpayTransResponse.h"
#include "payment_reader_input_args.h"
#include "reader_device_wrapper.h"
#include "mss_utils/ros_node_wrapper.h"
#include "worldpay_transactions_runner.h"


class PaymentReaderRosWrapper : public RosNodeWrapper
{
    public:
        PaymentReaderRosWrapper(PaymentReaderInputArgs in_args);

        void BeforeSpin(void) override;

        void AfterSpin(void) override;

        void ProcessSigint(void) override;

        void PublishWorldpayTransResponse(
                      mobile_payment_interface::WorldpayTransResponse response);

        void PublishCardTapEvent(bool bytes_found_in_card);

    private:
        const uint32_t kQueueSizeRosSubscriber = 1U;

        const uint32_t kQueueSizeRosPublisher = 10U;

        std::unique_ptr<ReaderDeviceWrapper> reader_device_;

        WorldpayTransactionsRunner transactions_runner_;

        PaymentReaderInputArgs in_args_;

        std::string PrependNodeNameToString(std::string topic);

        /* Transaction Account */
        const char* kTransactionAccountTopicName = "transaction_account";

        ros::Subscriber trans_account_ros_sub_;

        void TransAccountMsgCallback(
                             mobile_payment_interface::WorldpayAccount account);

        /* Transaction Amount */
        const char* kTransactionAmountTopicName = "transaction_amount";

        ros::Subscriber trans_amount_ros_sub_;

        void TransactionAmountMsgCallback(std_msgs::Float32 amount);

        /* Test Transaction Mode */
        const char* kTestTransModeTopicName = "test_transaction_mode";

        ros::Subscriber test_trans_mode_ros_sub_;

        void TestTransModeMsgCallback(std_msgs::Bool amount);

        /* Transaction Timeout */
        const char* kTransTimeoutTopicName = "transaction_timeout";

        ros::Subscriber trans_timeout_ros_sub_;

        void TransTimeoutMsgCallback(std_msgs::Int32 timeout);

        /* Sleep Time between Transactions */
        const char* kTransSleepTimeMsTopicName = "transaction_sleep_time_ms";

        ros::Subscriber trans_sleep_time_ros_sub_;

        void TransSleepTimeMsgCallback(std_msgs::Int32 time_ms);

        /* WorldPay Transaction Response */
        const char* kWorldpayTransResponseTopicName
                                              = "worldpay_transaction_response";

        ros::Publisher wp_tran_response_ros_pub_;

        /* WorldPay Transaction Response */
        const char* kCardTapEventTopicName = "card_tap_event";

        ros::Publisher card_tap_event_ros_pub_;

        void PublishMqttCardTapEventMessage(void);

        /* Container Lock Open Publisher */
        const char* kContainerLockOpenTopicName = "/lock_open_topic";

        ros::Publisher container_lock_open_ros_pub_;

        void OpenContainerLockIfApprovedTrans(
                      mobile_payment_interface::WorldpayTransResponse response);

        /* Transaction Enable Service */
        const char* kTransEnableServiceName = "transaction_enable";

        ros::ServiceServer trans_enable_service_;

        bool TransEnableServiceCallback(std_srvs::SetBool::Request &request,
                                        std_srvs::SetBool::Response &response);

        /* Mock Transaction Response Service */
        const char* kMockTransResponseServiceName = "mock_transaction_response";

        ros::ServiceServer mock_trans_response_service_;

        bool MockTransRespServiceCallback(std_srvs::Empty::Request &request,
                                          std_srvs::Empty::Response &response);

        /* SKU getter service client */
        const char* kSkuServiceName = "/sku";

        ros::ServiceClient sku_service_client_;

        std::string GetSkuFromRosService(void);

        /* MQTT message publisher  */
        const char* kMqttPubsTopicName = "/mqtt_publishers";

        ros::Publisher mqtt_msg_ros_pub_;

        void PublishMqttTransactionMessage(
                      mobile_payment_interface::WorldpayTransResponse response);

        /* Payment Reader connection status publisher */
        const char* kPayReaderConnectStatusServiceName
                                     = "/sensor_topic/payment_reader_connected";

        ros::ServiceServer pay_reader_connect_status_service_;

        bool PayReaderConnectStatusServiceCallback(
                                          std_srvs::Empty::Request &request,
                                          std_srvs::Empty::Response &response);

        /* Payment Reader Enabled Status */
        const char* kReaderEnabledStatusTopicName
                                       = "/sensor_topic/payment_reader_enabled";

        ros::Publisher reader_enabled_status_ros_pub_;

        void PublishPaymentReaderEnabledStatus(void);

        /* Payment Reader Test Mode Status */
        const char* kReaderTestModeStatusTopicName
                                     = "/sensor_topic/payment_reader_test_mode";

        ros::Publisher reader_test_mode_status_ros_pub_;

        void PublishPaymentTestModeStatus(void);
};
