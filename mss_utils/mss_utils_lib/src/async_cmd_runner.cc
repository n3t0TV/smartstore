#include <array>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

#include "async_cmd_runner.h"
#include "async_cmd_runner_exception.h"


void
AsyncCmdRunner::StartCmd(std::string command, int cmd_timeout_ms)
{
    JoinTimeoutThreadIfAny();
    StartCmdOncePrevTimeoutIfAnyIsDone(command, cmd_timeout_ms);
}


void
AsyncCmdRunner::StopCmd(void)
{
    JoinTimeoutThreadIfAny();
    StopCmdOnceTimeoutIsDone();
}


void
AsyncCmdRunner::AddCmdObserver(ICmdObserver *observer)
{
    list_observers_.push_back(observer);
}


void
AsyncCmdRunner::RmCmdObserver(ICmdObserver *observer)
{
    list_observers_.remove(observer);
}


AsyncCmdRunner::~AsyncCmdRunner(void)
{
    StopCmd();
    list_observers_.empty();
}


void
AsyncCmdRunner::JoinTimeoutThreadIfAny(void)
{
    if(cmd_timeout_stop_thread_)
    {
        /* Wake up the running timeout thread and wait until it exits */
        cmd_timeout_cond_var_.notify_all();
        cmd_timeout_stop_thread_->join();
        cmd_timeout_stop_thread_.reset();
    }
}


void
AsyncCmdRunner::StartCmdOncePrevTimeoutIfAnyIsDone(std::string command,
                                                   int cmd_timeout_ms)
{
    std::scoped_lock cmd_thread_lock(cmd_thread_mtx_);

    if(!cmd_thread_)
    {
        std::mutex cmd_pid_cond_var_mtx;
        std::unique_lock<std::mutex> cmd_pid_lock(cmd_pid_cond_var_mtx);

        cmd_pid_ = -1;
        command_ = command;
        cmd_thread_ = std::make_unique<std::thread>(&AsyncCmdRunner::Exec,
                                                    this);

        /* Wait until the PID of the command is known */
        cmd_pid_cond_var_.wait(cmd_pid_lock);

        /* If the PID is invalid, then the command couldn't be launch. Throw
         * exception to communicate error. */
        if(cmd_pid_ == -1)
        {
            cmd_thread_->join();
            cmd_thread_.reset();

            throw AsyncCmdRunnerException();
        }

        if(cmd_timeout_ms > kNoTimeout)
        {
            cmd_timeout_ms_ = cmd_timeout_ms;
            cmd_timeout_stop_thread_ = std::make_unique<std::thread>(
                                           &AsyncCmdRunner::StopCmdAfterTimeout,
                                           this);
        }
    }
}


void
AsyncCmdRunner::StopCmdOnceTimeoutIsDone(void)
{
    std::scoped_lock cmd_thread_lock(cmd_thread_mtx_);

    if(cmd_thread_)
    {
        pid_t process_group_id = -cmd_pid_;
        kill(process_group_id, SIGTERM);

        cmd_thread_->join();
        cmd_thread_.reset();
        cmd_pid_ = -1;
    }
}


void
AsyncCmdRunner::StopCmdAfterTimeout(void)
{
    std::mutex cmd_timeout_cond_var_mutex;
    std::unique_lock<std::mutex> lock(cmd_timeout_cond_var_mutex);
    auto chrono_sleep_time = std::chrono::milliseconds(cmd_timeout_ms_);
    std::cv_status timeout_status;

    timeout_status = cmd_timeout_cond_var_.wait_for(lock, chrono_sleep_time);

    if(timeout_status == std::cv_status::timeout)
    {
        StopCmdOnceTimeoutIsDone();
    }
}


void
AsyncCmdRunner::Exec(void)
{
    std::array<char, 512> buffer;
    pid_t pid = -1;
    pid_t ret_pid = -1;
    int pstat;

    try
    {
        std::unique_ptr<FILE, decltype(&fclose)> pipe(Popen(command_.c_str(),
                                                            "r", pid),
                                                      fclose);

        cmd_pid_ = pid;
        cmd_pid_cond_var_.notify_one();

        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        {
            std::string cmd_output = buffer.data();

            NotifyCmdObservers(cmd_output);
        }

        do
        {
            ret_pid = waitpid(pid, &pstat, 0);
        }
        while (ret_pid == -1 && errno == EINTR);
    }
    catch(const std::runtime_error& error)
    {
        cmd_pid_cond_var_.notify_one();
    }
}


/* Custom implementation of Popen that also returns the PID*/
FILE*
AsyncCmdRunner::Popen(const char* command, const char* type, pid_t& pid)
{
    int pipe_des[2];
    FILE* iop = NULL;

    if ((*type != 'r' && *type != 'w') || type[1] != '\0')
    {
        throw std::invalid_argument("Only 'r' and 'w' are supported as open"
                                    " types for Popen.");
    }

    if(pipe(pipe_des) < 0)
    {
        throw std::runtime_error("The pipe function failed during Popen.");
    }

    pid = fork();

    switch(pid)
    {
        case -1:
            (void) close(pipe_des[kPipeReadEnd]);
            (void) close(pipe_des[kPipeWriteEnd]);
            throw std::runtime_error("The fork function failed during Popen.");

        case 0:
            setpgid(getpid(), getpid());
            if(*type == 'r')
            {
                (void) close(pipe_des[kPipeReadEnd]);
                if(pipe_des[kPipeWriteEnd] != STDOUT_FILENO)
                {
                    dup2(pipe_des[kPipeWriteEnd], STDOUT_FILENO);
                    close(pipe_des[kPipeWriteEnd]);
                }
            }
            else
            {
                (void) close(pipe_des[kPipeWriteEnd]);
                if(pipe_des[kPipeReadEnd] != STDIN_FILENO)
                {
                    dup2(pipe_des[kPipeReadEnd], STDIN_FILENO);
                    close(pipe_des[kPipeReadEnd]);
                }
            }
            execl("/bin/sh", "sh", "-c", command, NULL);
            exit(1);
    }

    if(*type == 'r')
    {
        iop = fdopen(pipe_des[kPipeReadEnd], type);
        (void) close(pipe_des[kPipeWriteEnd]);
    }
    else
    {
        iop = fdopen(pipe_des[kPipeWriteEnd], type);
        (void) close(pipe_des[kPipeReadEnd]);
    }

    return iop;
}


void
AsyncCmdRunner::NotifyCmdObservers(std::string cmd_output)
{
    std::list<ICmdObserver *>::iterator iterator = list_observers_.begin();

    while(iterator != list_observers_.end())
    {
        (*iterator)->CmdOutputCallback(cmd_output);
        iterator++;
    }
}
