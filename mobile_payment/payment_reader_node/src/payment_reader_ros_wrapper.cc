#include "container/mqtt_publishers_msg.h"
#include "container/std_string.h"
#include "mqtt_transaction_stringifier.h"
#include "mss_utils/mss_date_time_utils.h"
#include "mss_utils/mss_ros_utils.h"
#include "payment_reader_ros_wrapper.h"
#include "reader_device_exception.h"
#include "reader_serial_getter.h"
#include "sku_getter.h"
#include "transaction_sleep_time.h"
#include "worldpay_account.h"
#include "worldpay_amount.h"
#include "worldpay_number_generator.h"
#include "worldpay_test_mode.h"
#include "worldpay_timeout.h"


PaymentReaderRosWrapper::PaymentReaderRosWrapper(
                                                PaymentReaderInputArgs in_args):
    in_args_(in_args), transactions_runner_(this)
{
}


void
PaymentReaderRosWrapper::BeforeSpin(void)
{
    ros::NodeHandle* node_hdl = GetNodeHandle();
    std::string topic_name;
    std::string sku_str;

    topic_name = PrependNodeNameToString(kTransactionAccountTopicName);
    trans_account_ros_sub_ = node_hdl->subscribe(topic_name,
                              kQueueSizeRosSubscriber,
                              &PaymentReaderRosWrapper::TransAccountMsgCallback,
                              this);

    topic_name = PrependNodeNameToString(kTransactionAmountTopicName);
    trans_amount_ros_sub_ = node_hdl->subscribe(topic_name,
                         kQueueSizeRosSubscriber,
                         &PaymentReaderRosWrapper::TransactionAmountMsgCallback,
                         this);

    topic_name = PrependNodeNameToString(kTestTransModeTopicName);
    test_trans_mode_ros_sub_ = node_hdl->subscribe(topic_name,
                             kQueueSizeRosSubscriber,
                             &PaymentReaderRosWrapper::TestTransModeMsgCallback,
                             this);

    topic_name = PrependNodeNameToString(kTransTimeoutTopicName);
    trans_timeout_ros_sub_ = node_hdl->subscribe(topic_name,
                              kQueueSizeRosSubscriber,
                              &PaymentReaderRosWrapper::TransTimeoutMsgCallback,
                              this);

    topic_name = PrependNodeNameToString(kTransSleepTimeMsTopicName);
    trans_sleep_time_ros_sub_ = node_hdl->subscribe(topic_name,
                            kQueueSizeRosSubscriber,
                            &PaymentReaderRosWrapper::TransSleepTimeMsgCallback,
                            this);

    topic_name = PrependNodeNameToString(kWorldpayTransResponseTopicName);
    wp_tran_response_ros_pub_
       = node_hdl->advertise<mobile_payment_interface::WorldpayTransResponse>(
                                      topic_name, kQueueSizeRosPublisher, true);

    topic_name = PrependNodeNameToString(kCardTapEventTopicName);
    card_tap_event_ros_pub_ = node_hdl->advertise<std_msgs::Bool>(topic_name,
                                                  kQueueSizeRosPublisher, true);

    container_lock_open_ros_pub_ = node_hdl->advertise<std_msgs::Bool>(
                                                 kContainerLockOpenTopicName,
                                                 kQueueSizeRosPublisher, false);

    topic_name = PrependNodeNameToString(kTransEnableServiceName);
    trans_enable_service_ = node_hdl->advertiseService(topic_name,
                           &PaymentReaderRosWrapper::TransEnableServiceCallback,
                           this);

    topic_name = PrependNodeNameToString(kMockTransResponseServiceName);
    mock_trans_response_service_ = node_hdl->advertiseService(topic_name,
                         &PaymentReaderRosWrapper::MockTransRespServiceCallback,
                         this);

    sku_service_client_ = node_hdl->serviceClient<container::std_string>(
                                                               kSkuServiceName);

    mqtt_msg_ros_pub_ = node_hdl->advertise<container::mqtt_publishers_msg>(
                                                         kMqttPubsTopicName,
                                                         kQueueSizeRosPublisher,
                                                         false);

    pay_reader_connect_status_service_ = node_hdl->advertiseService(
                kPayReaderConnectStatusServiceName,
                &PaymentReaderRosWrapper::PayReaderConnectStatusServiceCallback,
                this);

    reader_enabled_status_ros_pub_ = node_hdl->advertise<std_msgs::Bool>(
                                                  kReaderEnabledStatusTopicName,
                                                  kQueueSizeRosPublisher,
                                                  true);

    reader_test_mode_status_ros_pub_ = node_hdl->advertise<std_msgs::Bool>(
                                                 kReaderTestModeStatusTopicName,
                                                 kQueueSizeRosPublisher,
                                                 true);

    /* Notify the reader is disabled */
    PublishPaymentReaderEnabledStatus();

    /* Notify the test mode */
    PublishPaymentTestModeStatus();

    sku_str = GetSkuFromRosService();
    mss_worldpay::SetContainerSku(sku_str);

    reader_device_ = std::make_unique<ReaderDeviceWrapper>(in_args_);
    mss_worldpay::SetPaymentReaderSerial(
                                       reader_device_->GetCachedSerialNumber());

    transactions_runner_.StartTransactions();

    /* If the runner started, notify the reader is enabled */
    PublishPaymentReaderEnabledStatus();
}


void
PaymentReaderRosWrapper::AfterSpin(void)
{
    reader_device_.reset();
}


void
PaymentReaderRosWrapper::ProcessSigint(void)
{
    transactions_runner_.StopTransactions();
}


void
PaymentReaderRosWrapper::PublishWorldpayTransResponse(
                       mobile_payment_interface::WorldpayTransResponse response)
{
    wp_tran_response_ros_pub_.publish(response);
    OpenContainerLockIfApprovedTrans(response);
    PublishMqttTransactionMessage(response);
}


void
PaymentReaderRosWrapper::PublishCardTapEvent(bool bytes_found_in_card)
{
    std_msgs::Bool bool_msg;

    bool_msg.data = bytes_found_in_card;

    card_tap_event_ros_pub_.publish(bool_msg);
    PublishMqttCardTapEventMessage();
}


void
PaymentReaderRosWrapper::PublishMqttCardTapEventMessage(void)
{
    const char* kMqttTransTopic = "card_tap";
    const int kDeliverMqttMsgAtLeastOnce = 1;
    const char* kEmptyCardTapMsg = "{}";
    container::mqtt_publishers_msg mqtt_pubs_msg;

    mqtt_pubs_msg.mqtt_topic = kMqttTransTopic;
    mqtt_pubs_msg.raw_msg = kEmptyCardTapMsg;
    mqtt_pubs_msg.qos = kDeliverMqttMsgAtLeastOnce;

    mqtt_msg_ros_pub_.publish(mqtt_pubs_msg);
}


std::string
PaymentReaderRosWrapper::PrependNodeNameToString(std::string str)
{
    std::stringstream prepend_topic;

    prepend_topic << GetNodeName() << "/" << str;

    return prepend_topic.str();
}


void
PaymentReaderRosWrapper::TransAccountMsgCallback(
                              mobile_payment_interface::WorldpayAccount account)
{
    if(account.use_test_account && account.use_tortoise_account)
    {
        MSS_ROS_WARN("Both use_test_account and use_tortoise_account are set:"
                     " please set only one");
    }
    else
    {
        if(account.use_test_account)
        {
            account.id = account.kTestAccountId;
            account.token = account.kTestAccountToken;
            account.acceptor_id = account.kTestAcceptorId;
        }
        else if(account.use_tortoise_account)
        {
            account.id = account.kTortoiseAccountId;
            account.token = account.kTortoiseAccountToken;
            account.acceptor_id = account.kTortoiseAcceptorId;
        }

        mss_worldpay::UpdateAccount(account);
        transactions_runner_.CancelCurrentTransactions();
    }
}


void
PaymentReaderRosWrapper::TransactionAmountMsgCallback(std_msgs::Float32 amount)
{
    mss_worldpay::UpdateTransactionAmount(amount.data);
    transactions_runner_.CancelCurrentTransactions();
}


void
PaymentReaderRosWrapper::TestTransModeMsgCallback(std_msgs::Bool test_enabled)
{
    mss_worldpay::UpdateTestMode(test_enabled.data);
    transactions_runner_.CancelCurrentTransactions();

    PublishPaymentTestModeStatus();
}


void
PaymentReaderRosWrapper::TransTimeoutMsgCallback(std_msgs::Int32 timeout)
{
    mss_worldpay::UpdateTransactionTimeout(timeout.data);
    transactions_runner_.CancelCurrentTransactions();
}


void
PaymentReaderRosWrapper::TransSleepTimeMsgCallback(std_msgs::Int32 time_ms)
{
    mss_worldpay::UpdateTransactionSleepTime(time_ms.data);
}


bool
PaymentReaderRosWrapper::TransEnableServiceCallback(
                                          std_srvs::SetBool::Request &request,
                                          std_srvs::SetBool::Response &response)
{
    if(request.data == true)
    {
        transactions_runner_.StartTransactions();
    }
    else
    {
        transactions_runner_.StopTransactions();
    }

    PublishPaymentReaderEnabledStatus();

    return true;
}


bool
PaymentReaderRosWrapper::MockTransRespServiceCallback(
                                            std_srvs::Empty::Request &request,
                                            std_srvs::Empty::Response &response)
{
    mobile_payment_interface::WorldpayTransResponse wp_trans_response;

    transactions_runner_.StopTransactions();

    wp_trans_response.express_response_code
                                      = wp_trans_response.kApprovedResponseCode;
    wp_trans_response.express_response_message = "Approved";

    wp_trans_response.express_transaction_date
                               = mss_date_time_utils::GetDateInYyyymmddFormat();
    wp_trans_response.express_transaction_time
                                 = mss_date_time_utils::GetTimeInHhmmssFormat();
    wp_trans_response.express_transaction_timezone
                                           = mss_date_time_utils::GetTimeZone();

    wp_trans_response.transaction_id
                              = std::to_string(mss_worldpay::GetRandomNumber());
    wp_trans_response.approval_number
                              = std::to_string(mss_worldpay::GetRandomNumber());

    wp_trans_response.reference_number
                           = mss_worldpay::GetReferenceNumberWithSkuAndSerial();
    mss_worldpay::IncrementReferenceNumber();
    mss_worldpay::IncrementTicketNumber();

    wp_trans_response.approved_amount = mss_worldpay::GetTransactionAmount();

    wp_trans_response.is_mock_response = true;

    PublishWorldpayTransResponse(wp_trans_response);

    transactions_runner_.StartTransactions();

    return true;
}


std::string
PaymentReaderRosWrapper::GetSkuFromRosService(void)
{
    std::string sku_str;
    container::std_string sku_srv_msg;

    if(sku_service_client_.call(sku_srv_msg))
    {
        sku_str = sku_srv_msg.response.data;
    }
    else
    {
        throw std::runtime_error("Calling the service to obtain the SKU"
                                 " failed: the service could be down");
    }

    return sku_str;
}


void
PaymentReaderRosWrapper::OpenContainerLockIfApprovedTrans(
                       mobile_payment_interface::WorldpayTransResponse response)
{
    if(response.express_response_code == response.kApprovedResponseCode)
    {
        std_msgs::Bool lock_bool;

        lock_bool.data = true;
        container_lock_open_ros_pub_.publish(lock_bool);
    }
}


void
PaymentReaderRosWrapper::PublishMqttTransactionMessage(
                       mobile_payment_interface::WorldpayTransResponse response)
{
    const char* kMqttTransTopic = "transaction";
    const int kDeliverMqttMsgAtLeastOnce = 1;
    std::string container_sku = mss_worldpay::GetContainerSku();
    std::string reader_serial_num = reader_device_->GetCachedSerialNumber();
    auto mqtt_trans_strfier = MqttTransactionStringifier(response,
                                                         container_sku,
                                                         reader_serial_num);
    container::mqtt_publishers_msg mqtt_pubs_msg;

    mqtt_pubs_msg.mqtt_topic = kMqttTransTopic;
    mqtt_pubs_msg.raw_msg = mqtt_trans_strfier.GetMqttTransactionString();
    mqtt_pubs_msg.qos = kDeliverMqttMsgAtLeastOnce;

    mqtt_msg_ros_pub_.publish(mqtt_pubs_msg);
}


bool
PaymentReaderRosWrapper::PayReaderConnectStatusServiceCallback(
                                            std_srvs::Empty::Request &request,
                                            std_srvs::Empty::Response &response)
{
    bool connect_status;

    try
    {
        reader_device_->VerifyCurrentReaderIsAttached();
        connect_status = true;
    }
    catch(const ReaderDeviceException& err)
    {
        MSS_ROS_ERROR("The payment reader device is not attached: %s",
                      err.what());
        connect_status = false;
    }

    return connect_status;
}


void
PaymentReaderRosWrapper::PublishPaymentReaderEnabledStatus(void)
{
    std_msgs::Bool enabled_status;

    enabled_status.data = transactions_runner_.AreTransactionsRunning();
    reader_enabled_status_ros_pub_.publish(enabled_status);
}


void
PaymentReaderRosWrapper::PublishPaymentTestModeStatus(void)
{
    std_msgs::Bool test_mode_status;

    test_mode_status.data =  mss_worldpay::GetTestMode();
    reader_test_mode_status_ros_pub_.publish(test_mode_status);
}
