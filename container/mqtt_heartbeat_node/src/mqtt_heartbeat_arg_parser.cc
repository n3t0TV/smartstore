#include <cstring>
#include <stdexcept>

#include "mqtt_heartbeat_arg_parser.h"


MqttHeartbeatArgParser::MqttHeartbeatArgParser(int argc, char **argv)
{
    error_t parse_error = 0;

    argp_options_.push_back({0});

    memset(&arg_parser_, 0, sizeof(arg_parser_));
    arg_parser_.options = &argp_options_[0];
    arg_parser_.parser = &MqttHeartbeatArgParser::ArgpParseOption;

    parse_error = argp_parse(&arg_parser_, argc, argv, ARGP_NO_EXIT, 0,
                             &input_args_);

    if(parse_error != 0)
    {
        throw std::invalid_argument("The input arguments couldn't be parsed or"
                                    " they're not expected");
    }
}


error_t
MqttHeartbeatArgParser::ArgpParseOption(int option_key, char *option_value,
                                  struct argp_state *state)
{
    error_t retval = 0;
    MqttHeartbeatInputArgs* in_args = (MqttHeartbeatInputArgs*) state->input;

    switch (option_key)
    {
        case ARGP_KEY_ARG:
            {
                std::string sarg = option_value;
                std::string node_name_key = "__name:=";

                if(sarg.rfind(node_name_key, 0) == 0)
                {
                    sarg.erase(0, node_name_key.length());
                    in_args->node_name = sarg;
                }

                break;
            }
        case ARGP_KEY_END:
            {
                break;
            }
        default:
            {
                retval = ARGP_ERR_UNKNOWN;
            }
    }

    return retval;
}


MqttHeartbeatInputArgs
MqttHeartbeatArgParser::GetInputArgs(void)
{
    return input_args_;
}
