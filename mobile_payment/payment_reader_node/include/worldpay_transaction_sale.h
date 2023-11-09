#pragma once

#include "worldpay_transaction.h"


class WorldpayTransactionSale : public WorldpayTransaction
{
    public:
        WorldpayTransactionSale(mss_worldpay::TransactionCommon common,
                                float amount);
};
