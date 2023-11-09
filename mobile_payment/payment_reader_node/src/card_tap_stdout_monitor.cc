#include <ros/console.h>

#include "card_tap_monitor_exception.h"
#include "card_tap_stdout_monitor.h"
#include "mss_utils/mss_ros_utils.h"
#include "payment_reader_ros_wrapper.h"
#include "stdout_to_file.h"
#include "stdout_to_file_exception.h"


CardTapStdoutMonitor::CardTapStdoutMonitor(PaymentReaderRosWrapper* wrapper):
    ros_wrapper_(wrapper)
{
}


void
CardTapStdoutMonitor::StartMonitoring(void)
{
    std::scoped_lock monitor_thread_lock(monitor_thread_mtx_);

    if(!monitor_thread_)
    {
        MSS_ROS_INFO("Starting monitoring card tap by reading STDOUT");
        stop_monitoring_  = false;
        monitor_thread_ = std::make_unique<std::thread>(
                                             &CardTapStdoutMonitor::MonitorLoop,
                                             this);
    }
    else
    {
        MSS_ROS_WARN("Request to start monitoring card tap was ignored: "
                     "monitoring already running");
    }
}


void
CardTapStdoutMonitor::StopMonitoring(void)
{
    std::scoped_lock monitor_thread_lock(monitor_thread_mtx_);

    if(monitor_thread_)
    {
        stop_cond_var_.notify_all();
        stop_monitoring_  = true;
        monitor_thread_->join();

        MSS_ROS_INFO("The card tap monitoring using STDOUT was stopped");

        monitor_thread_.reset();
    }
    else
    {
        MSS_ROS_WARN("Request to stop monitoring card tap was ignored: "
                     "monitoring already stopped");
    }
}


void
CardTapStdoutMonitor::MonitorLoop(void)
{
    try
    {
        std::mutex stop_cond_var_mutex;
        std::unique_lock<std::mutex> lock(stop_cond_var_mutex);
        StdoutToFile stdout_to_file_;
        std::string out_file_path = stdout_to_file_.GetFilePath();

        stdout_file_stream_ = std::ifstream(out_file_path.c_str());

        while(!stop_monitoring_)
        {
            (void) stop_cond_var_.wait_for(lock,
                                           std::chrono::milliseconds(50));
            (void) fflush(stdout);
            FindAllTagStringsAndResetStreamIfFailed();
        }
    }
    catch(const StdoutToFileException& err)
    {
        MSS_ROS_ERROR("Couldn't redirect STDOUT to detect the tap of the card"
                      " on the payment reader: %s", err.what());
    }
}


void
CardTapStdoutMonitor::FindAllTagStringsAndResetStreamIfFailed(void)
{
    try
    {
        bool card_has_bytes = false;

        FindContactlessTagsInStreamAndUpdatePos();
        FindUnencryptedTags();
        card_has_bytes = FindByteTags();
        ros_wrapper_->PublishCardTapEvent(card_has_bytes);
        stop_monitoring_ = true;
    }
    catch(const CardTapMonitorException& err)
    {
        stdout_file_stream_.clear();
        stdout_file_stream_.seekg(prev_stream_pos_, std::ios::beg);
    }
}


void
CardTapStdoutMonitor::FindContactlessTagsInStreamAndUpdatePos(void)
{
    std::string stream_line;
    bool found = false;

    while(stdout_file_stream_)
    {
        prev_stream_pos_ = stdout_file_stream_.tellg();
        std::getline(stdout_file_stream_, stream_line);

        if(stream_line.rfind(kContactlessTagsString, 0) == 0)
        {
            found = true;
            break;
        }
    }

    if(!found)
    {
        throw CardTapMonitorException();
    }
}


void
CardTapStdoutMonitor::FindUnencryptedTags(void)
{
    std::string stream_line;
    bool found = false;

    if(stdout_file_stream_)
    {
        std::getline(stdout_file_stream_, stream_line);

        if(stream_line.rfind(kUnencryptedTagsString, 0) == 0)
        {
            found = true;
        }
    }

    if(!found)
    {
        throw CardTapMonitorException();
    }
}


bool
CardTapStdoutMonitor::FindByteTags(void)
{
    std::string stream_line;
    bool found = false;
    bool bytes_present = false;

    if(stdout_file_stream_)
    {
        std::getline(stdout_file_stream_, stream_line);

        if(stream_line.rfind(kByteCountString, 0) == 0)
        {
            bytes_present = true;
            found = true;
        }
        else if(stream_line.rfind(kNoBytesString, 0) == 0)
        {
            bytes_present = false;
            found = true;
        }
    }

    if(!found)
    {
        throw CardTapMonitorException();
    }

    return bytes_present;
}
