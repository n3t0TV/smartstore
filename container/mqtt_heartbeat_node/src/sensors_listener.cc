#include <std_srvs/Empty.h>

#include "mqtt_heartbeat_ros_wrapper.h"


SensorsListener::SensorsListener(MqttHeartbeatRosWrapper* hb_ros):
    hb_ros_(hb_ros)
{
}


void
SensorsListener::InitializeRosObjects(void)
{
    ros::NodeHandle* node_hdl = hb_ros_->GetNodeHandle();

    battery_level_ros_sub_ = node_hdl->subscribe(kBatteryLevelTopicName,
                                         kQueueSizeRosSubscriber,
                                         &SensorsListener::BatteryLevelCallback,
                                         this);

    lid_open_close_ros_sub_ = node_hdl->subscribe(kLidOpenCloseTopicName,
                                         kQueueSizeRosSubscriber,
                                         &SensorsListener::LidOpenCloseCallback,
                                         this);

    gps_location_ros_sub_ = node_hdl->subscribe(kGpsLocationTopicName,
                                          kQueueSizeRosSubscriber,
                                          &SensorsListener::GpsLocationCallback,
                                          this);

    /* The payment reader connection status is polled from the sensor node
     * instead of consuming the status from the payment reader node because
     * the reader node might die and not report a disconnection status. */
    pay_reader_connect_status_service_client_ = node_hdl
         ->serviceClient<std_srvs::Empty>(kPayReaderConnectedStatusServiceName);

    pay_reader_enabled_status_ros_sub_ = node_hdl->subscribe(
                           kPayReaderEnabledStatusTopicName,
                           kQueueSizeRosSubscriber,
                           &SensorsListener::PaymentReaderEnabledStatusCallback,
                           this);

    pay_reader_test_mode_status_ros_sub_ = node_hdl->subscribe(
                          kPayReaderTestModeStatusTopicName,
                          kQueueSizeRosSubscriber,
                          &SensorsListener::PaymentReaderTestModeStatusCallback,
                          this);

    /* Initialize the sensors JSON */
    sensors_json_[kBattLvlJsonKey] = 0;
    sensors_json_[kLidStatusJsonKey] = false;
    sensors_json_[kGpsLocationJsonKey] = {{kGpsLatJsonKey, 0.0},
                                          {kGpsLonJsonKey, 0.0},
                                          {kGpsAltJsonKey, 0.0}};
    UpdatePaymentReaderConnectedStatus();
    sensors_json_[kPayReaderEnabledJsonKey] = false;
    sensors_json_[kPayReaderTestModeJsonKey] = false;
}


nlohmann::json
SensorsListener::GetSensorsJson(void)
{
    UpdatePaymentReaderConnectedStatus();

    return sensors_json_;
}


void
SensorsListener::BatteryLevelCallback(const std_msgs::Int32& msg)
{
    sensors_json_[kBattLvlJsonKey] = msg.data;
}


void
SensorsListener::LidOpenCloseCallback(const std_msgs::Bool& msg)
{
    sensors_json_[kLidStatusJsonKey] = msg.data ? true : false;

    hb_ros_->UpdateAndPublishSensorJson(sensors_json_);
}


void
SensorsListener::GpsLocationCallback(const container::gps_msg& msg)
{
    double lat = msg.Latitude;
    double lon = msg.Longitude;
    double alt = msg.Altitude;

    nlohmann::json j = {{kGpsLatJsonKey, lat}, {kGpsLonJsonKey, lon},
                        {kGpsAltJsonKey, alt}};

    sensors_json_[kGpsLocationJsonKey] = j;

    /* There's no need to publish the latest GPS location. It will be
     * published at the rate the heartbeat. */
}


void
SensorsListener::UpdatePaymentReaderConnectedStatus(void)
{
    std_srvs::Empty empty_msg;

    bool con_status = pay_reader_connect_status_service_client_.call(empty_msg);
    sensors_json_[kPayReaderConnectedJsonKey] = con_status;

    /* If the connection status is false, the reader is disabled */
    if(!con_status)
    {
        sensors_json_[kPayReaderEnabledJsonKey] = false;
        sensors_json_[kPayReaderTestModeJsonKey] = false;
    }
}


void
SensorsListener::PaymentReaderEnabledStatusCallback(const std_msgs::Bool& msg)
{
    sensors_json_[kPayReaderEnabledJsonKey] = msg.data ? true : false;

    /* If the enabled status is true, the reader is connected */
    if(msg.data)
    {
        sensors_json_[kPayReaderConnectedJsonKey] = true;
    }

    hb_ros_->UpdateAndPublishSensorJson(sensors_json_);
}


void
SensorsListener::PaymentReaderTestModeStatusCallback(const std_msgs::Bool& msg)
{
    sensors_json_[kPayReaderTestModeJsonKey] = msg.data ? true : false;
    hb_ros_->UpdateAndPublishSensorJson(sensors_json_);
}
