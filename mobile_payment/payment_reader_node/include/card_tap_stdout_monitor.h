#pragma once

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <thread>


class PaymentReaderRosWrapper;


class CardTapStdoutMonitor
{
    public:
        CardTapStdoutMonitor(PaymentReaderRosWrapper* ros_wrapper);

        void StartMonitoring(void);

        void StopMonitoring(void);

    private:
        const char* kContactlessTagsString = "CONTACTLESS Unencrypted Tags:";

        const char* kUnencryptedTagsString = "Unencrypted Tags:";

        const char* kByteCountString = "Byte count = ";

        const char* kNoBytesString = "NO Bytes";

        std::atomic<bool> stop_monitoring_ = false;

        std::mutex monitor_thread_mtx_;

        std::unique_ptr<std::thread> monitor_thread_;

        std::condition_variable stop_cond_var_;

        std::ifstream stdout_file_stream_;

        PaymentReaderRosWrapper* ros_wrapper_;

        int prev_stream_pos_ = 0;

        void MonitorLoop(void);

        void FindAllTagStringsAndResetStreamIfFailed(void);

        void FindContactlessTagsInStreamAndUpdatePos(void);

        void FindUnencryptedTags(void);

        bool FindByteTags(void);
};
