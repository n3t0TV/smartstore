#include <ros/package.h>

#include "mss_utils/mss_ros_utils.h"
#include "speaker_ros_wrapper.h"


SpeakerRosWrapper::SpeakerRosWrapper(SpeakerInputArgs in_args):
    in_args_(in_args)
{
}


void
SpeakerRosWrapper::BeforeSpin(void)
{
    const char* kAudiosDir = "/speaker_audios";
    ros::NodeHandle* node_hdl = GetNodeHandle();
    std::string topic_name;

    wp_trans_response_ros_sub_ = node_hdl->subscribe(
                           kWorldpayTransResponseTopicName,
                           kQueueSizeRosSubscriber,
                           &SpeakerRosWrapper::WorldPayTransResponseMsgCallback,
                           this);
    audios_dir_path_ = (ros::package::getPath(mss_ros_utils::GetPackageName())
                        + kAudiosDir);

    container_lock_open_ros_sub_ = node_hdl->subscribe(
                                    kCardTapEventTopicName,
                                    kQueueSizeRosSubscriber,
                                    &SpeakerRosWrapper::CardTapEventMsgCallback,
                                    this);

    topic_name = PrependNodeNameToString(kSetVolumeTopicName);
    set_vol_ros_sub_ = node_hdl->subscribe(topic_name, kQueueSizeRosSubscriber,
                                       &SpeakerRosWrapper::SetVolumeMsgCallback,
                                       this);

    topic_name = PrependNodeNameToString(kGetVolumeTopicName);
    get_vol_ros_sub_ = node_hdl->advertise<std_msgs::Int32>(topic_name,
                                                  kQueueSizeRosPublisher, true);
}


void
SpeakerRosWrapper::AfterSpin(void)
{
}


void
SpeakerRosWrapper::ProcessSigint(void)
{
}


void
SpeakerRosWrapper::WorldPayTransResponseMsgCallback(
                 mobile_payment_interface::WorldpayTransResponse trans_response)
{
    const char* kApprovedAudio = "success.mp3";
    const char* kNotApprovedAudio = "fail.mp3";
    std::string file_path;

    if(trans_response.express_response_code
       == trans_response.kApprovedResponseCode)
    {
        file_path = audios_dir_path_ + "/" + kApprovedAudio;
        MSS_ROS_INFO("Playing %s", kApprovedAudio);
    }
    else
    {
        file_path = audios_dir_path_ + "/" + kNotApprovedAudio;
        MSS_ROS_INFO("Playing %s", kNotApprovedAudio);
    }

    mpg123_player_.Play(file_path);
}


void
SpeakerRosWrapper::CardTapEventMsgCallback(std_msgs::Bool bytes_found_in_card)
{
    const char* kLoadingAudio = "loading.mp3";
    std::string file_path;

    MSS_ROS_INFO("Playing loading");
    file_path = audios_dir_path_ + "/" + kLoadingAudio;

    mpg123_player_.PlayLoop(file_path);
}


void
SpeakerRosWrapper::SetVolumeMsgCallback(std_msgs::Int32 vol_level)
{
    try
    {
        mpg123_player_.SetPercentageVolume(vol_level.data);
        get_vol_ros_sub_.publish(vol_level);
    }
    catch(const std::invalid_argument& err)
    {
        MSS_ROS_ERROR("Couldn't set volume: %s", err.what());
    }
}


std::string
SpeakerRosWrapper::PrependNodeNameToString(std::string str)
{
    std::stringstream prepend_topic;

    prepend_topic << GetNodeName() << "/" << str;

    return prepend_topic.str();
}
