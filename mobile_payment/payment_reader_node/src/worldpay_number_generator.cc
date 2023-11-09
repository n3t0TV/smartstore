#include <random>
#include <ros/ros.h>

#include "idtech/IDTDef.h"
#include "mss_utils/mss_ros_utils.h"
#include "mss_utils/persistence_reader_writer.h"
#include "reader_serial_getter.h"
#include "sku_getter.h"
#include "worldpay_number_generator.h"

static const char* kLastReferenceNumberKey = "last_reference_num";

static int reference_number = 0;
static bool ref_num_previously_read = false;
static std::recursive_mutex ref_num_mutex;

static const char* kLastTicketNumberKey = "last_ticket_num";

static int ticket_number = 0;
static bool ticket_num_previously_read = false;
static std::recursive_mutex ticket_num_mutex;


void
mss_worldpay::IncrementReferenceNumber(void)
{
    auto& persistence = PersistenceReaderWriter::GetInstance();
    std::scoped_lock ref_num_lock(ref_num_mutex);

    if(!ref_num_previously_read)
    {
        (void) GetReferenceNumber();
    }

    reference_number++;
    persistence.WriteInt(kLastReferenceNumberKey, reference_number);
    persistence.Dump();
}


int
mss_worldpay::GetReferenceNumber(void)
{
    auto& persist = PersistenceReaderWriter::GetInstance();
    std::scoped_lock ref_num_lock(ref_num_mutex);

    try
    {
        if(!ref_num_previously_read)
        {
            reference_number = persist.ReadInt(kLastReferenceNumberKey);
            ref_num_previously_read = true;
        }
    }
    catch(const std::invalid_argument& err)
    {
        MSS_ROS_WARN("Starting at reference number zero: %s", err.what());
        reference_number = 0;
        persist.WriteInt(kLastReferenceNumberKey, reference_number);
        persist.Dump();
    }

    return reference_number;
}


std::string
mss_worldpay::GetReferenceNumberWithSkuAndSerial(void)
{
    const int kDashCharSize = 2;
    WorldPayData worldpay_data;
    /* Leave one character for null end */
    int ref_num_size = ((sizeof(worldpay_data.referenceNumber)
                         / sizeof(worldpay_data.referenceNumber[0])) - 1);
    std::string sku = mss_worldpay::GetContainerSku();
    std::string serial_num = mss_worldpay::GetPaymentReaderSerial();
    std::stringstream reference_stream;

    reference_stream << sku << "-" << serial_num << "-"
                     << std::setw(ref_num_size - sku.size() - serial_num.size()
                                  - kDashCharSize)
                     << std::setfill('0') << GetReferenceNumber();

    return reference_stream.str();
}


void
mss_worldpay::IncrementTicketNumber(void)
{
    auto& persistence = PersistenceReaderWriter::GetInstance();
    std::scoped_lock ticket_num_lock(ticket_num_mutex);

    if(!ticket_num_previously_read)
    {
        (void) GetTicketNumber();
    }

    ticket_number++;
    persistence.WriteInt(kLastTicketNumberKey, ticket_number);
    persistence.Dump();
}


int
mss_worldpay::GetTicketNumber(void)
{
    auto& persist = PersistenceReaderWriter::GetInstance();
    std::scoped_lock ticket_num_lock(ticket_num_mutex);

    try
    {
        if(!ticket_num_previously_read)
        {
            ticket_number = persist.ReadInt(kLastTicketNumberKey);
            ticket_num_previously_read = true;
        }
    }
    catch(const std::invalid_argument& err)
    {
        MSS_ROS_WARN("Starting at ticket number zero: %s", err.what());
        ticket_number = 0;
        persist.WriteInt(kLastTicketNumberKey, ticket_number);
        persist.Dump();
    }

    return ticket_number;
}


int
mss_worldpay::GetRandomNumber(void)
{
    const int kRandomNumLower = 100'000;
    const int kRandomNumUpper = 999'999;
    auto gen = std::mt19937{std::random_device{}()};
    auto dist = std::uniform_int_distribution<int>(kRandomNumLower,
                                                   kRandomNumUpper);
    return dist(gen);
}
