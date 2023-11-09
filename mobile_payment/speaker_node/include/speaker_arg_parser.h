#pragma once

#include <argp.h>
#include <vector>

#include "speaker_input_args.h"


class SpeakerArgParser
{
    public:
        SpeakerArgParser(int argc, char **argv);

        SpeakerInputArgs GetInputArgs(void);

    private:
        std::vector<struct argp_option> argp_options_;

        struct argp arg_parser_;

        SpeakerInputArgs input_args_;

        static error_t ArgpParseOption(int option_key, char *option_value,
                                       struct argp_state *state);
};
