#include "speaker_arg_parser.h"
#include "speaker_input_args.h"
#include "speaker_ros_wrapper.h"


int main(int argc, char **argv)
{
    try
    {
        const std::uint16_t kMasterTimeoutSeconds = 30U;
        SpeakerArgParser parser(argc, argv);
        SpeakerInputArgs in_args = parser.GetInputArgs();
        SpeakerRosWrapper spaker_ros(in_args);

        spaker_ros.Loop(argc, argv, in_args.node_name, kMasterTimeoutSeconds);
    }
    catch(const std::invalid_argument& error)
    {
        printf("[speaker_node] Invalid arguments: %s\n", error.what());
    }
}
