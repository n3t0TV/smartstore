#pragma once

#include <string>


namespace mss_worldpay {
    extern void SetPaymentReaderSerial(std::string reader_serial);

    extern std::string GetPaymentReaderSerial(void);
}
