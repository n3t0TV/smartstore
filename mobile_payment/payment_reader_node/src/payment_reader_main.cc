#include "payment_reader_arg_parser.h"
#include "payment_reader_ros_wrapper.h"


int main(int argc, char **argv)
{
    try
    {
        const std::uint16_t kMasterTimeoutSeconds = 30U;
        PaymentReaderArgParser parser(argc, argv);
        PaymentReaderInputArgs in_args = parser.GetInputArgs();
        PaymentReaderRosWrapper reader_ros(in_args);

        reader_ros.Loop(argc, argv, in_args.node_name, kMasterTimeoutSeconds);
    }
    catch(const std::invalid_argument& error)
    {
        printf("[payment_reader_node] Invalid arguments: %s\n", error.what());
    }
}
