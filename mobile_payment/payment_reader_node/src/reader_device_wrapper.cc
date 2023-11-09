#include <sstream>

#include "idtech/libIDT_Device.h"
#include "mss_utils/mss_file_system_utils.h"
#include "reader_device_exception.h"
#include "reader_device_wrapper.h"
#include "worldpay_utils.h"


ReaderDeviceWrapper::ReaderDeviceWrapper(PaymentReaderInputArgs in_args):
    in_args_(in_args)
{
    try
    {
        InitReaderDevice();
        VerifyReaderIsConnected();
        SetReaderCertificatesPath();
        SetCurrentReaderDeviceModel();
        VerifyCurrentReaderIsAttached();
        SetPollOnDemand();
    }
    catch(const std::exception& ex)
    {
        CloseReaderDevice();
        throw;
    }
}


std::string
ReaderDeviceWrapper::GetCachedSerialNumber(void)
{
    if(serial_number_cached_.empty())
    {
        serial_number_cached_ = GetSerialNumber();
    }

    return serial_number_cached_;
}


ReaderDeviceWrapper::~ReaderDeviceWrapper(void)
{
    CloseReaderDevice();
}


void
ReaderDeviceWrapper::InitReaderDevice(void)
{
    int dev_err = device_init();

    if(dev_err != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The device initialization failed: "
                       << mss_worldpay::DeviceErrCodeToString(dev_err);

        throw ReaderDeviceException(err_msg_stream.str());
    }
    else
    {
        is_dev_initialized_ = true;
    }
}


void
ReaderDeviceWrapper::VerifyReaderIsConnected(void)
{
    if(device_isConnected() == 0)
    {
        throw ReaderDeviceException("The device is not connected");
    }
}


void
ReaderDeviceWrapper::SetReaderCertificatesPath(void)
{
    int dev_err = 0;
    const int kReaderIsProductionUnit = 1;

    VerifyCertsFileExists();
    dev_err = device_startRKI(in_args_.certs_folder_path.c_str(),
                              kReaderIsProductionUnit);

    if(dev_err != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't set the certificates path: "
                       << mss_worldpay::DeviceErrCodeToString(dev_err);

        throw ReaderDeviceException(err_msg_stream.str());
    }
}


void
ReaderDeviceWrapper::SetCurrentReaderDeviceModel(void)
{
    int set_ok = device_setCurrentDevice(kIdTechDevType);

    if(set_ok == 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Couldn't set " << kIdTechDevName
                       << " as the current reader device";

        throw ReaderDeviceException(err_msg_stream.str());
    }
}


void
ReaderDeviceWrapper::VerifyCurrentReaderIsAttached(void)
{
    if(device_isAttached(kIdTechDevType) == 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The current reader device " << kIdTechDevName
                       << " is not attached";

        throw ReaderDeviceException(err_msg_stream.str());
    }
}


void
ReaderDeviceWrapper::SetPollOnDemand(void)
{
    const int kPollOnDemandMode = 1;
    int dev_err = device_setPollMode(kPollOnDemandMode);

    if(dev_err != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "Setting the poll mode on demand for the reader"
                       << " device failed: "
                       << mss_worldpay::DeviceErrCodeToString(dev_err);

        throw ReaderDeviceException(err_msg_stream.str());
    }
}


std::string
ReaderDeviceWrapper::GetSerialNumber(void)
{
    int serial_num_len = kSerialNumCharLen;
    int dev_code = config_getSerialNumber_Len(serial_number_char_,
                                              &serial_num_len);

    if(dev_code != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The device serial number couldn't be obtained: "
                       << mss_worldpay::DeviceResponseCodeToString(dev_code);

        throw ReaderDeviceException(err_msg_stream.str());
    }

    return serial_number_char_;
}


void
ReaderDeviceWrapper::CloseReaderDevice(void)
{
    if(is_dev_initialized_ && device_close() != 0)
    {
        throw ReaderDeviceException("The reader device couldn't be closed"
                                    " successfully");
    }
}


void
ReaderDeviceWrapper::VerifyCertsFileExists(void)
{
    const char* kCertsFileName = "ca-certificates.crt";
    std::string certs_file_path = in_args_.certs_folder_path + kCertsFileName;

    if(!mss_fs_utils::FileExists(certs_file_path))
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The certificates file is not present at "
                       << certs_file_path;

        throw ReaderDeviceException(err_msg_stream.str());
    }
}
