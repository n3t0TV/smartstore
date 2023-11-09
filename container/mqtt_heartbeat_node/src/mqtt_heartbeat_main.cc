#include "mqtt_heartbeat_arg_parser.h"
#include "mqtt_heartbeat_ros_wrapper.h"


int main(int argc, char **argv)
{
    try
    {
        const std::uint16_t kMasterTimeoutSeconds = 30U;
        MqttHeartbeatArgParser parser(argc, argv);
        MqttHeartbeatInputArgs in_args = parser.GetInputArgs();
        MqttHeartbeatRosWrapper heartbeat_ros;

        heartbeat_ros.Loop(argc, argv, in_args.node_name,
                           kMasterTimeoutSeconds);
    }
    catch(const std::invalid_argument& error)
    {
        printf("[mqtt_heartbeat_node] Invalid arguments: %s\n", error.what());
    }
}
