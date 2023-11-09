#pragma once

#include <cstring>

#include "idtech/IDTDef.h"
#include "worldpay_account.h"
#include "worldpay_transaction_common.h"


class WorldpayTransaction
{
    public:
        WorldpayTransaction(mss_worldpay::TransactionCommon wpt_common);

        WorldPayData GetWorldPayData(void);

    protected:
        WorldPayData worldpay_data_;

        void SetTransactionAmount(float amout);

    private:
        mss_worldpay::TransactionCommon wpt_common_;

        void UpdateWorldpayAccount(void);

        void SetReferenceNumber(void);

        void SetTicketNumber(void);

        void SetTerminalId(void);

        void SetTestTransaction(void);
};
