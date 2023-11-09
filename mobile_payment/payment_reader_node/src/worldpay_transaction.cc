#include <iomanip>
#include <sstream>

#include "worldpay_timeout.h"
#include "worldpay_transaction.h"


WorldpayTransaction::WorldpayTransaction(
                                    mss_worldpay::TransactionCommon wpt_common):
    wpt_common_(wpt_common)
{
    static const int kContaclesssOn = 1;
    static const int kMagneticStripeReaderOff = 0;
    static const int kDuplicateCheckOff = 0;
    static const int kDuplicateChargesOn = 1;

    /* Only set fields that are common for all the transaction types here. Let
     * child classes for each specific transaction fill the rest as they
     * require. */
    memset(&worldpay_data_, 0, sizeof(worldpay_data_));
    worldpay_data_.enableCTLS = kContaclesssOn;
    worldpay_data_.msrOnly = kMagneticStripeReaderOff;
    worldpay_data_.duplicateCheck = kDuplicateCheckOff;
    worldpay_data_.duplicateOverride = kDuplicateChargesOn;

    /* TODO: Will move timeout of here once code is more settle */
    worldpay_data_.timeout = mss_worldpay::GetTransactionTimeout();

    UpdateWorldpayAccount();
    SetReferenceNumber();
    SetTicketNumber();
    SetTerminalId();
    SetTestTransaction();
}


WorldPayData
WorldpayTransaction::GetWorldPayData(void)
{
    return worldpay_data_;
}


void
WorldpayTransaction::UpdateWorldpayAccount(void)
{
    auto& wp_account = wpt_common_.account;

    strcpy(worldpay_data_.accountID, wp_account.id.c_str());
    strcpy(worldpay_data_.accountToken, wp_account.token.c_str());
    strcpy(worldpay_data_.acceptorID, wp_account.acceptor_id.c_str());
}


void
WorldpayTransaction::SetReferenceNumber(void)
{
    strcpy(worldpay_data_.referenceNumber,
           wpt_common_.reference_number.c_str());
}


void
WorldpayTransaction::SetTicketNumber(void)
{
    /* Leave one character for null end */
    int ticket_num_size = ((sizeof(worldpay_data_.ticketNumber)
                            / sizeof(worldpay_data_.ticketNumber[0])) - 1);
    std::stringstream ticket_stream;

    ticket_stream << std::setw(ticket_num_size) << std::setfill('0')
                  << wpt_common_.ticket_number;
    strcpy(worldpay_data_.ticketNumber, ticket_stream.str().c_str());
}


void
WorldpayTransaction::SetTerminalId(void)
{
    strcpy(worldpay_data_.terminalID, wpt_common_.terminal_id.c_str());
}


void
WorldpayTransaction::SetTestTransaction(void)
{
    worldpay_data_.isTest = (wpt_common_.is_test ? 1 : 0);
}


void
WorldpayTransaction::SetTransactionAmount(float amout)
{
    static const int kAmountPrecision = 2;
    std::stringstream amount_stream;

    amount_stream << std::fixed << std::setprecision(kAmountPrecision) << amout;

    strcpy(worldpay_data_.amount, amount_stream.str().c_str());
}
