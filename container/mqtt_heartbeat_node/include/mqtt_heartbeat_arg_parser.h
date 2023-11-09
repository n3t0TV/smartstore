#pragma once

#include <argp.h>
#include <vector>

#include "mqtt_heartbeat_input_args.h"


class MqttHeartbeatArgParser
{
    public:
        MqttHeartbeatArgParser(int argc, char **argv);

        MqttHeartbeatInputArgs GetInputArgs(void);

    private:
        std::vector<struct argp_option> argp_options_;

        struct argp arg_parser_;

        MqttHeartbeatInputArgs input_args_;

        static error_t ArgpParseOption(int option_key, char *option_value,
                                       struct argp_state *state);
};
