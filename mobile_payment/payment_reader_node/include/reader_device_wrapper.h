#pragma once

#include <string>

#include "idtech/IDTDef.h"
#include "payment_reader_input_args.h"


class ReaderDeviceWrapper
{
    public:
        ReaderDeviceWrapper(PaymentReaderInputArgs in_args);

        std::string GetCachedSerialNumber(void);

        void VerifyCurrentReaderIsAttached(void);

        ~ReaderDeviceWrapper(void);

    private:
        const DEVICE_TYPE kIdTechDevType = IDT_DEVICE_VP3300_USB;

        const char* kIdTechDevName = "IDT_DEVICE_VP3300_USB";

        static const int kSerialNumCharLen = 64;

        bool is_dev_initialized_ = false;

        PaymentReaderInputArgs in_args_;

        char serial_number_char_[kSerialNumCharLen];

        std::string serial_number_cached_;

        void InitReaderDevice(void);

        void VerifyReaderIsConnected(void);

        void SetReaderCertificatesPath(void);

        void SetCurrentReaderDeviceModel(void);

        void SetPollOnDemand(void);

        std::string GetSerialNumber(void);

        void CloseReaderDevice(void);

        void VerifyCertsFileExists(void);
};
