#include "worldpay_transaction_sale.h"


WorldpayTransactionSale::WorldpayTransactionSale(
                                         mss_worldpay::TransactionCommon common,
                                         float amount):
    WorldpayTransaction(common)
{
    worldpay_data_.transactionType = TRANSACTION_TYPE_SALE;

    SetTransactionAmount(amount);
}
