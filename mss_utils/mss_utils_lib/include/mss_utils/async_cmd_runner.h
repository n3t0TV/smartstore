#pragma once

#include <atomic>
#include <condition_variable>
#include <list>
#include <mutex>
#include <string>
#include <thread>


class ICmdObserver
{
    public:
        virtual void CmdOutputCallback(std:: string cmd_output) = 0;
};


class AsyncCmdRunner
{
    public:
        void StartCmd(std::string command, int cmd_timeout_ms=kNoTimeout);

        void StopCmd(void);

        void AddCmdObserver(ICmdObserver *observer);

        void RmCmdObserver(ICmdObserver *observer);

        ~AsyncCmdRunner(void);

    private:
        static const int kNoTimeout = 0;

        const int kPipeReadEnd = 0;

        const int kPipeWriteEnd = 1;

        int cmd_timeout_ms_ = 0;

        std::string command_;

        std::list<ICmdObserver *> list_observers_;

        std::atomic<pid_t> cmd_pid_;

        std::mutex cmd_thread_mtx_;

        std::unique_ptr<std::thread> cmd_thread_;

        std::unique_ptr<std::thread> cmd_timeout_stop_thread_;

        std::condition_variable cmd_pid_cond_var_;

        std::condition_variable cmd_timeout_cond_var_;

        void JoinTimeoutThreadIfAny(void);

        void StartCmdOncePrevTimeoutIfAnyIsDone(std::string command,
                                                int cmd_timeout_ms);

        void StopCmdOnceTimeoutIsDone(void);

        void StopCmdAfterTimeout(void);

        void Exec(void);

        FILE* Popen(const char* command, const char* type, pid_t& pid);

        void NotifyCmdObservers(std::string cmd_output);
};
