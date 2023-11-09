#include <chrono>
#include <mutex>
#include <ros/ros.h>

#include "idtech/libIDT_Device.h"
#include "mss_utils/mss_ros_utils.h"
#include "payment_reader_ros_wrapper.h"
#include "transaction_sleep_time.h"
#include "worldpay_data_factory.h"
#include "worldpay_number_generator.h"
#include "worldpay_parser_exception.h"
#include "worldpay_response_parser.h"
#include "worldpay_timeout.h"
#include "worldpay_transactions_runner.h"
#include "worldpay_utils.h"


std::condition_variable WorldpayTransactionsRunner::stop_cond_var_;

std::atomic<bool> WorldpayTransactionsRunner::transaction_failed_ = false;

PaymentReaderRosWrapper* WorldpayTransactionsRunner::ros_wrapper_;


WorldpayTransactionsRunner::WorldpayTransactionsRunner(
                                          PaymentReaderRosWrapper* ros_wrapper):
    card_stdout_tap_monitor_(ros_wrapper)
{
    ros_wrapper_ = ros_wrapper;
}


void
WorldpayTransactionsRunner::StartTransactions(void)
{
    std::scoped_lock trans_thread_lock(trans_thread_mtx_);

    if(!trans_thread_)
    {
        MSS_ROS_INFO("Starting WorldPay Transactions");
        transactions_stopped_ = false;
        trans_thread_ = std::make_unique<std::thread>(
                                  &WorldpayTransactionsRunner::TransactionsLoop,
                                  this);
    }
    else
    {
        MSS_ROS_WARN("Request to Start Transactions ignored: transactions"
                     " already running");
    }
}


void
WorldpayTransactionsRunner::CancelCurrentTransactions(void)
{
    std::scoped_lock trans_thread_lock(trans_thread_mtx_);

    if(trans_thread_)
    {
        int cancel_ok = ctls_cancelTransaction();

        if(cancel_ok != 0)
        {
            std::stringstream err_msg_stream;

            err_msg_stream << "The cancellation of the WorldPay transaction"
                << " failed: "
                << mss_worldpay::DeviceErrCodeToString(cancel_ok);
            MSS_ROS_ERROR("%s", err_msg_stream.str().c_str());
        }
    }
    else
    {
        MSS_ROS_WARN("Request to Cancel Current Transaction ignored:"
                     " transactions already stopped");
    }
}


void
WorldpayTransactionsRunner::StopTransactions(void)
{
    std::scoped_lock trans_thread_lock(trans_thread_mtx_);

    if(trans_thread_)
    {
        int cancel_ok = 0;

        transactions_stopped_ = true;

        cancel_ok = ctls_cancelTransaction();
        if(cancel_ok != 0)
        {
            std::stringstream err_msg_stream;

            err_msg_stream << "The cancellation of the WorldPay transaction"
                           << " failed: "
                           << mss_worldpay::DeviceErrCodeToString(cancel_ok);
            MSS_ROS_ERROR("%s", err_msg_stream.str().c_str());
        }

        stop_cond_var_.notify_all();
        trans_thread_->join();

        MSS_ROS_INFO("WorldPay Transactions stopped");

        worldpay_data_uptr_.reset();
        trans_thread_.reset();
    }
    else
    {
        MSS_ROS_WARN("Request to Stop Transactions ignored: transactions"
                     " already stopped");
    }
}


bool
WorldpayTransactionsRunner::AreTransactionsRunning(void)
{
    std::scoped_lock trans_thread_lock(trans_thread_mtx_);
    return (trans_thread_ != nullptr);
}


void
WorldpayTransactionsRunner::TransactionsLoop(void)
{
    WorldpayDataFactory wp_data_factory_;
    auto wp_data = wp_data_factory_.GetWorldpayData(TRANSACTION_TYPE_SALE);
    std::mutex stop_cond_var_mutex;
    std::unique_lock<std::mutex> lock(stop_cond_var_mutex);
    auto rest_time = std::chrono::milliseconds(kRestTimeMsBetweenTransactions);

    ctls_registerCallBkp(&ContactlessTransCallback);

    while(!transactions_stopped_)
    {
        const int kReturnRequestAndResponse = 1;
        int exe_ok = 0;

        wp_data = wp_data_factory_.GetWorldpayData(TRANSACTION_TYPE_SALE);
        worldpay_data_uptr_.reset();
        worldpay_data_uptr_ = std::make_unique<WorldPayData>(wp_data);

        transaction_failed_ = false;
        exe_ok = executeTransaction(worldpay_data_uptr_.get(),
                                  &WorldpayTransactionsRunner::WorldPayCallback,
                                  kReturnRequestAndResponse);
        if(exe_ok != 0)
        {
            std::stringstream err_msg_stream;

            err_msg_stream << "The transaction execution failed: "
                           << mss_worldpay::DeviceErrCodeToString(exe_ok);
            MSS_ROS_ERROR("%s", err_msg_stream.str().c_str());
        }
        else
        {
            int cond_timeout = (mss_worldpay::GetTransactionTimeout()
                                + kAdditionalCallbackTimeoutSeconds);
            std::cv_status timeout_status;

            card_stdout_tap_monitor_.StartMonitoring();

            timeout_status = stop_cond_var_.wait_for(lock,
                                            std::chrono::seconds(cond_timeout));

            card_stdout_tap_monitor_.StopMonitoring();

            if(timeout_status == std::cv_status::timeout)
            {
                std::scoped_lock trans_thread_lock(trans_thread_mtx_);

                MSS_ROS_WARN("The WorldPay callback didn't timeout itself,"
                             " restarting the transaction.");

                int cancel_ok = ctls_cancelTransaction();

                if(cancel_ok != 0)
                {
                    std::stringstream err_msg_stream;

                    err_msg_stream << "The cancellation of the WorldPay"
                        << " transaction failed: "
                        << mss_worldpay::DeviceErrCodeToString(cancel_ok);
                    MSS_ROS_ERROR("%s", err_msg_stream.str().c_str());
                }
            }

            if(!transactions_stopped_ && !transaction_failed_)
            {
                int sleep_time = mss_worldpay::GetTransactionSleepTime();
                auto chrono_sleep_time = std::chrono::milliseconds(sleep_time);

                mss_worldpay::IncrementReferenceNumber();
                mss_worldpay::IncrementTicketNumber();

                stop_cond_var_.wait_for(lock, chrono_sleep_time);
            }
        }

        if(!transactions_stopped_)
        {
            std::this_thread::sleep_for(rest_time);
        }
    }
}


void
WorldpayTransactionsRunner::WorldPayCallback(char* message, int data,
                                             int result)
{

    try
    {
        WorldpayResponseParser parser;
        auto response = parser.Parse(message);
        transaction_failed_ = true;

        MSS_ROS_INFO("express_response_code: %d",
                     response.express_response_code);
        MSS_ROS_INFO("express_response_message: %s",
                     response.express_response_message.c_str());
        MSS_ROS_INFO("reference_number: %s", response.reference_number.c_str());

        if(response.express_response_code == response.kApprovedResponseCode)
        {
            transaction_failed_ = false;
            MSS_ROS_INFO("express_transaction_date: %s",
                         response.express_transaction_date.c_str());
            MSS_ROS_INFO("express_transaction_time: %s",
                         response.express_transaction_time.c_str());
            MSS_ROS_INFO("express_transaction_timezone: %s",
                         response.express_transaction_timezone.c_str());
            MSS_ROS_INFO("transaction_id: %s",
                         response.transaction_id.c_str());
            MSS_ROS_INFO("approval_number: %s",
                         response.approval_number.c_str());
            MSS_ROS_INFO("approved_amount: %.2f", response.approved_amount);
        }

        stop_cond_var_.notify_all();
        ros_wrapper_->PublishWorldpayTransResponse(response);

    }
    catch(const WorldpayParserException& error)
    {
        mobile_payment_interface::WorldpayTransResponse failed_response;

        MSS_ROS_ERROR("Couldnt parse XML: %s", error.what());
        MSS_ROS_INFO("%s", message);

        failed_response.express_response_code = RETURN_CODE_ERR_CMD_RESPONSE;
        failed_response.express_response_message = "No response found in the "
                                                   "returned XML from WorldPay";
        failed_response.reference_number =
                             mss_worldpay::GetReferenceNumberWithSkuAndSerial();

        MSS_ROS_INFO("express_response_code: %d",
                     failed_response.express_response_code);
        MSS_ROS_INFO("express_response_message: %s",
                     failed_response.express_response_message.c_str());
        MSS_ROS_INFO("reference_number: %s",
                     failed_response.reference_number.c_str());

        transaction_failed_ = true;
        stop_cond_var_.notify_all();
        ros_wrapper_->PublishWorldpayTransResponse(failed_response);
    }

    (void) fflush(stdout);
}


void
WorldpayTransactionsRunner::ContactlessTransCallback(int type,
                                                     IDTMSRData* cardData1)
{
    switch (type)
    {
        case MSR_callBack_type_ERR:
            {
                MSS_ROS_INFO("Contactless Callback: Card Swipe/Tap Cancelled");
                transaction_failed_ = true;
                stop_cond_var_.notify_all();
                break;
            }
        case MSR_callBack_type_RETURN_CODE:
            {
                MSS_ROS_INFO("Contactless Callback: RETURN_CODE");
                break;
            }
        case MSR_callBack_type_TIMEOUT:
            {
                MSS_ROS_INFO("Contactless Callback: Timeout");
                transaction_failed_ = true;
                stop_cond_var_.notify_all();
                break;
            }
        case MSR_callBack_type_CARD_READ_ERR:
            {
                MSS_ROS_INFO("Contactless Callback: Card Read Error");
                break;
            }
        case MSR_callBack_type_TERMINATED:
            {
                MSS_ROS_INFO("Contactless Callback: Terminated");
                break;
            }
        case MSR_callBack_type_FALLBACK_TO_CONTACT:
            {
                MSS_ROS_INFO("Contactless Callback: Fallback to contact");
                break;
            }
        case MSR_callBack_type_CORE_DUMP:
            {
                MSS_ROS_INFO("Contactless Callback: CORE_DUMP");
                break;
            }
        case MSR_callBack_type_ERR_CODE:
            {
                if(cardData1->errorCode == IDG_P2_STATUS_CODE_TIMEOUT)
                {
                    MSS_ROS_INFO("Contactless Callback: Transaction Timed-out");
                }
                else if(cardData1->errorCode
                        == IDG_P2_STATUS_CODE_COMMAND_NOT_ALLOWED)
                {
                    MSS_ROS_INFO("Contactless Callback: Transaction Command not"
                                 " allowed");
                }

                else
                {
                    MSS_ROS_INFO("Contactless Callback: Error code 0x%04x",
                                 cardData1->errorCode);
                }

                transaction_failed_ = true;
                stop_cond_var_.notify_all();
                break;
            }
        default:
            {
                break;
            }
    }
    (void) fflush(stdout);
}
