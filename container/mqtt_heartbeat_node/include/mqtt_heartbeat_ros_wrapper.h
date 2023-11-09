#pragma once

#include <mutex>
#include <std_msgs/String.h>

#include "mss_utils/json.hpp"
#include "mss_utils/ros_node_wrapper.h"
#include "sensors_listener.h"


class MqttHeartbeatRosWrapper : public RosNodeWrapper
{
    public:
        MqttHeartbeatRosWrapper(void);

        void BeforeSpin(void) override;

        void AfterSpin(void) override;

        void ProcessSigint(void) override;

        void UpdateAndPublishSensorJson(nlohmann::json sensor_json);

    private:
        const uint32_t kQueueSizeRosSubscriber = 10U;

        const uint32_t kQueueSizeRosPublisher = 10U;

        const char* kMqttHbSensorsKey = "sensors";

        const char* kMqttHbSkuKey = "sku";

        const char* kMqttHbVersionKey = "version";

        const char* kMqttHbProviderKey = "provider";

        const double kHeartbeatTimerSeconds = 10.0;

        const double kRosServiceTimeoutSeconds = 2.0;

        std::mutex update_sensor_mutex_;

        nlohmann::json mqtt_heartbeat_json_;

        SensorsListener sensors_listener_;

        std::string PrependNodeNameToString(std::string topic);

        /* Callback triggered by timer to publish MQTT Heartbeat */
        ros::Timer heartbeat_ros_timer_;

        void HeartbeatTimerCallback(const ros::TimerEvent& event);

        /* MQTT message publisher  */
        const char* kMqttPubsTopicName = "/mqtt_publishers";

        ros::Publisher mqtt_pubs_ros_pub_;

        void PublishMqttSensorMessage(void);

        /* Sensors subscriber */
        const char* kMqttSensorTopicName = "/sensor_topic";

        ros::Subscriber mqtt_sensor_ros_sub_;

        void SensorMsgCallback(const std_msgs::String& sensor_msg);

        /* SKU getter service client */
        const char* kSkuServiceName = "/sku";

        ros::ServiceClient sku_service_client_;

        void GetSkuFromRosService(void);

        /* Version getter service client */
        const char* kVersionServiceName = "/version";

        ros::ServiceClient version_service_client_;

        void GetVersionFromRosService(void);

        /* Provider getter service client */
        const char* kProviderServiceName = "/provider";

        ros::ServiceClient provider_service_client_;

        void GetProviderFromRosService(void);
};
