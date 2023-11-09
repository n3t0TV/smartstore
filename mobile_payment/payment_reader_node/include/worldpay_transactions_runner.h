#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <thread>

#include "card_tap_stdout_monitor.h"
#include "idtech/IDTDef.h"


class PaymentReaderRosWrapper;


class WorldpayTransactionsRunner
{
    public:
        WorldpayTransactionsRunner(PaymentReaderRosWrapper* ros_wrapper);

        void StartTransactions(void);

        void CancelCurrentTransactions(void);

        void StopTransactions(void);

        bool AreTransactionsRunning(void);

    private:
        const int kAdditionalCallbackTimeoutSeconds = 5;

        const int kRestTimeMsBetweenTransactions = 500;

        std::atomic<bool> transactions_stopped_ = true;

        static std::atomic<bool> transaction_failed_;

        std::mutex trans_thread_mtx_;

        std::unique_ptr<std::thread> trans_thread_;

        std::unique_ptr<WorldPayData> worldpay_data_uptr_;

        static std::condition_variable stop_cond_var_;

        CardTapStdoutMonitor card_stdout_tap_monitor_;

        static PaymentReaderRosWrapper* ros_wrapper_;

        void TransactionsLoop(void);

        static void WorldPayCallback(char* message, int data, int result);

        static void ContactlessTransCallback(int type, IDTMSRData* cardData1);
};
