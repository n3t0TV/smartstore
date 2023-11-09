#pragma once

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include "container/gps_msg.h"


class MqttHeartbeatRosWrapper;


class SensorsListener
{
    public:
        SensorsListener(MqttHeartbeatRosWrapper* hb_ros);

        void InitializeRosObjects(void);

        nlohmann::json GetSensorsJson(void);

    private:
        const uint32_t kQueueSizeRosSubscriber = 10U;

        MqttHeartbeatRosWrapper* hb_ros_;

        nlohmann::json sensors_json_;

        /* Battery Level subscriber  */
        const char* kBattLvlJsonKey = "battery";

        const char* kBatteryLevelTopicName = "sensor_topic/battery";

        ros::Subscriber battery_level_ros_sub_;

        void BatteryLevelCallback(const std_msgs::Int32& msg);

        /* Lid Open/Close status subscriber  */
        const char* kLidStatusJsonKey = "lid";

        const char* kLidOpenCloseTopicName = "sensor_topic/lid";

        ros::Subscriber lid_open_close_ros_sub_;

        void LidOpenCloseCallback(const std_msgs::Bool& msg);

        /* GPS subscriber */
        const char* kGpsLatJsonKey = "lat";

        const char* kGpsLonJsonKey = "lon";

        const char* kGpsAltJsonKey = "alt";

        const char* kGpsLocationJsonKey = "gps";

        const char* kGpsLocationTopicName = "sensor_topic/gps";

        ros::Subscriber gps_location_ros_sub_;

        void GpsLocationCallback(const container::gps_msg& msg);

        /* Payment Reader Connected Status service client */
        const char* kPayReaderConnectedJsonKey = "payment_reader_connected";

        const char* kPayReaderConnectedStatusServiceName
                                     = "/sensor_topic/payment_reader_connected";

        ros::ServiceClient pay_reader_connect_status_service_client_;

        void UpdatePaymentReaderConnectedStatus(void);

        /* Payment Reader Enabled Status subscriber */
        const char* kPayReaderEnabledJsonKey = "payment_reader_enabled";

        const char* kPayReaderEnabledStatusTopicName
                                       = "/sensor_topic/payment_reader_enabled";

        ros::Subscriber pay_reader_enabled_status_ros_sub_;

        void PaymentReaderEnabledStatusCallback(const std_msgs::Bool& msg);

        /* Payment Reader Test Mode Status subscriber */
        const char* kPayReaderTestModeJsonKey = "payment_reader_test_mode";

        const char* kPayReaderTestModeStatusTopicName
                                     = "/sensor_topic/payment_reader_test_mode";

        ros::Subscriber pay_reader_test_mode_status_ros_sub_;

        void PaymentReaderTestModeStatusCallback(const std_msgs::Bool& msg);
};
