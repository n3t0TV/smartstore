#include <stdexcept>

#include "sku_getter.h"
#include "worldpay_account.h"
#include "worldpay_amount.h"
#include "worldpay_data_factory.h"
#include "worldpay_number_generator.h"
#include "worldpay_test_mode.h"
#include "worldpay_transaction_sale.h"


WorldPayData
WorldpayDataFactory::GetWorldpayData(TRANSACTION_TYPE transaction_type)
{
    WorldPayData wp_data;

    AssembleWorldpayTransactionCommon();

    switch(transaction_type)
    {
        case TRANSACTION_TYPE_SALE:
            {
                wp_data = AssembleWorldpayTransactionSaleData();
                break;
            }
        default:
            {
                throw std::invalid_argument("WorldPay transaction type not"
                                            " implemented");
                break;
            }
    }

    return wp_data;
}


void
WorldpayDataFactory::AssembleWorldpayTransactionCommon(void)
{
    wpt_common_.account = mss_worldpay::GetAccount();
    wpt_common_.reference_number
                           = mss_worldpay::GetReferenceNumberWithSkuAndSerial();
    wpt_common_.ticket_number = mss_worldpay::GetTicketNumber();
    wpt_common_.terminal_id = mss_worldpay::GetContainerSku();
    wpt_common_.is_test = mss_worldpay::GetTestMode();
}


WorldPayData
WorldpayDataFactory::AssembleWorldpayTransactionSaleData(void)
{
    float amount = mss_worldpay::GetTransactionAmount();
    auto sale = WorldpayTransactionSale(wpt_common_, amount);

    return sale.GetWorldPayData();
}
