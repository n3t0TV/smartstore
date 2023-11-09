#include "container/mqtt_publishers_msg.h"
#include "container/std_string.h"
#include "mqtt_heartbeat_ros_wrapper.h"


MqttHeartbeatRosWrapper::MqttHeartbeatRosWrapper(void):
    sensors_listener_(this)
{
}


void
MqttHeartbeatRosWrapper::BeforeSpin(void)
{
    ros::NodeHandle* node_hdl = GetNodeHandle();
    std::string topic_name;

    mqtt_pubs_ros_pub_ = node_hdl->advertise<container::mqtt_publishers_msg>(
                                                         kMqttPubsTopicName,
                                                         kQueueSizeRosPublisher,
                                                         false);

    mqtt_sensor_ros_sub_ = node_hdl->subscribe(
                                    kMqttSensorTopicName,
                                    kQueueSizeRosSubscriber,
                                    &MqttHeartbeatRosWrapper::SensorMsgCallback,
                                    this);

    sku_service_client_ = node_hdl->serviceClient<container::std_string>(
                                                               kSkuServiceName);

    version_service_client_ = node_hdl->serviceClient<container::std_string>(
                                                           kVersionServiceName);

    provider_service_client_ = node_hdl->serviceClient<container::std_string>(
                                                           kMqttHbProviderKey);

    heartbeat_ros_timer_ = node_hdl->createTimer(
                               ros::Duration(kHeartbeatTimerSeconds),
                               &MqttHeartbeatRosWrapper::HeartbeatTimerCallback,
                               this);

    try
    {
        GetSkuFromRosService();
        GetVersionFromRosService();
        GetProviderFromRosService();
        sensors_listener_.InitializeRosObjects();
    }
    catch(std::runtime_error& error)
    {
        ROS_ERROR("Couldn't initialize the heartbeat data: %s", error.what());
    }
}


void
MqttHeartbeatRosWrapper::AfterSpin(void)
{
}


void
MqttHeartbeatRosWrapper::ProcessSigint(void)
{
}


void
MqttHeartbeatRosWrapper::UpdateAndPublishSensorJson(nlohmann::json sensor_json)
{
    /* This update can be called on demand from sensors or timer, that's why
     * mutex is required. */
    std::scoped_lock update_sensor_lock(update_sensor_mutex_);

    mqtt_heartbeat_json_[kMqttHbSensorsKey] = sensor_json;

    PublishMqttSensorMessage();
}


std::string
MqttHeartbeatRosWrapper::PrependNodeNameToString(std::string str)
{
    std::stringstream prepend_topic;

    prepend_topic << GetNodeName() << "/" << str;

    return prepend_topic.str();
}


void
MqttHeartbeatRosWrapper::HeartbeatTimerCallback(const ros::TimerEvent& event)
{
    auto sensors_json = sensors_listener_.GetSensorsJson();

    try
    {
        GetProviderFromRosService();
    }
    catch(std::runtime_error& error)
    {
        ROS_ERROR("Provider service error: %s", error.what());
    }

    UpdateAndPublishSensorJson(sensors_json);
}


void
MqttHeartbeatRosWrapper::PublishMqttSensorMessage(void)
{
    const char* kMqttSensorTopic = "sensor";
    const int kDeliverMqttMsgAtLeastOnce = 1;
    container::mqtt_publishers_msg mqtt_pubs_msg;

    mqtt_pubs_msg.mqtt_topic = kMqttSensorTopic;
    mqtt_pubs_msg.raw_msg = mqtt_heartbeat_json_.dump();
    mqtt_pubs_msg.qos = kDeliverMqttMsgAtLeastOnce;

    mqtt_pubs_ros_pub_.publish(mqtt_pubs_msg);
}


void
MqttHeartbeatRosWrapper::SensorMsgCallback(const std_msgs::String& sensor_msg)
{
    auto sensors_json = nlohmann::json::parse(sensor_msg.data);
    mqtt_heartbeat_json_[kMqttHbSensorsKey] = sensors_json;

    PublishMqttSensorMessage();
}


void
MqttHeartbeatRosWrapper::GetSkuFromRosService(void)
{
    container::std_string sku_srv_msg;

    ros::service::waitForService(kSkuServiceName,
                                 ros::Duration(kRosServiceTimeoutSeconds));

    if(sku_service_client_.call(sku_srv_msg))
    {
        mqtt_heartbeat_json_[kMqttHbSkuKey] = sku_srv_msg.response.data;
    }
    else
    {
        throw std::runtime_error("Calling the service to obtain the SKU"
                                 " failed: the service could be down");
    }
}


void
MqttHeartbeatRosWrapper::GetVersionFromRosService(void)
{
    container::std_string version_srv_msg;

    ros::service::waitForService(kVersionServiceName,
                                 ros::Duration(kRosServiceTimeoutSeconds));

    if(version_service_client_.call(version_srv_msg))
    {
        mqtt_heartbeat_json_[kMqttHbVersionKey] = version_srv_msg.response.data;
    }
    else
    {
        throw std::runtime_error("Calling the service to obtain the Version"
                                 " failed: the service could be down");
    }
}


void
MqttHeartbeatRosWrapper::GetProviderFromRosService(void)
{
    container::std_string provider_srv_msg;

    ros::service::waitForService(kProviderServiceName,
                                 ros::Duration(kRosServiceTimeoutSeconds));

    if(provider_service_client_.call(provider_srv_msg))
    {
        mqtt_heartbeat_json_[kMqttHbProviderKey] = provider_srv_msg.response.data;
    }
    else
    {
        throw std::runtime_error("Calling the service to obtain the Provider"
                                 " failed: the service could be down");
    }
}
