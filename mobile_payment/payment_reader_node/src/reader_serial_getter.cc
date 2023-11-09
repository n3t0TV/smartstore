#include "reader_serial_getter.h"

static std::string reader_serial_;


void
mss_worldpay::SetPaymentReaderSerial(std::string reader_serial)
{
    reader_serial_ = reader_serial;
}


std::string
mss_worldpay::GetPaymentReaderSerial(void)
{
    return reader_serial_;
}
