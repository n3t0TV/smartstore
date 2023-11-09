#pragma once

#include <string>


namespace mss_worldpay {
    void IncrementReferenceNumber(void);

    int GetReferenceNumber(void);

    std::string GetReferenceNumberWithSkuAndSerial(void);

    void IncrementTicketNumber(void);

    int GetTicketNumber(void);

    int GetRandomNumber(void);
}
