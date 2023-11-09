#pragma once

#include <argp.h>
#include <vector>

#include "payment_reader_input_args.h"


class PaymentReaderArgParser
{
    public:
        PaymentReaderArgParser(int argc, char **argv);

        PaymentReaderInputArgs GetInputArgs(void);

    private:
        static const char kCertsFolderPathOptionKey = 'c';

        const argp_option kCertsFolderPathOption = {"certs-folder-path",
                                                    kCertsFolderPathOptionKey,
                                                    "CERTS-FOLDER-PATH", 0,
                                                    "Folder containing the"
                                                    " certificates file for the"
                                                    " payment reader."};

        std::vector<struct argp_option> argp_options_;

        struct argp arg_parser_;

        PaymentReaderInputArgs input_args_;

        static error_t ArgpParseOption(int option_key, char *option_value,
                                       struct argp_state *state);
};
