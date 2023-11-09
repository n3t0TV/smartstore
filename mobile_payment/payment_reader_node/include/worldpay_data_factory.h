#pragma once

#include "idtech/IDTDef.h"
#include "worldpay_transaction_common.h"


class WorldpayDataFactory
{
    public:
        WorldPayData GetWorldpayData(TRANSACTION_TYPE transaction_type);

    private:
        mss_worldpay::TransactionCommon wpt_common_;

        void AssembleWorldpayTransactionCommon(void);

        WorldPayData AssembleWorldpayTransactionSaleData(void);
};
