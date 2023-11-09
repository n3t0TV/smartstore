#pragma once

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include "mobile_payment_interface/WorldpayTransResponse.h"
#include "mpg123_cmd_player.h"
#include "mss_utils/ros_node_wrapper.h"
#include "speaker_input_args.h"


class SpeakerRosWrapper : public RosNodeWrapper
{
    public:
        SpeakerRosWrapper(SpeakerInputArgs in_args);

        void BeforeSpin(void) override;

        void AfterSpin(void) override;

        void ProcessSigint(void) override;

    private:
        const uint32_t kQueueSizeRosSubscriber = 10U;

        const uint32_t kQueueSizeRosPublisher = 10U;

        SpeakerInputArgs in_args_;

        std::string audios_dir_path_;

        Mpg123Player mpg123_player_;

        std::string PrependNodeNameToString(std::string topic);

        /* WorldPay Transaction Response */
        const char* kWorldpayTransResponseTopicName
                              = "/payment_reader/worldpay_transaction_response";

        ros::Subscriber wp_trans_response_ros_sub_;

        void WorldPayTransResponseMsgCallback(
                mobile_payment_interface::WorldpayTransResponse trans_response);

        /* Card Tap Events */
        const char* kCardTapEventTopicName = "/payment_reader/card_tap_event";

        ros::Subscriber container_lock_open_ros_sub_;

        void CardTapEventMsgCallback(std_msgs::Bool bytes_found_in_card);

        /* Set Volume Subscriber */
        const char* kSetVolumeTopicName = "set_volume";

        ros::Subscriber set_vol_ros_sub_;

        void SetVolumeMsgCallback(std_msgs::Int32 vol_level);

        /* Get Volume Publisher */
        const char* kGetVolumeTopicName = "get_volume";

        ros::Publisher get_vol_ros_sub_;
};
