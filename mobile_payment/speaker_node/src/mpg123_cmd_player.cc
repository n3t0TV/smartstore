#include <cmath>
#include <ros/ros.h>

#include "mpg123_cmd_player.h"
#include "mss_utils/async_cmd_runner_exception.h"
#include "mss_utils/mss_ros_utils.h"


void
Mpg123Player::Play(std::string audio_file_path)
{
    std::string cmd = GetCmd(audio_file_path);
    int exit_code = 0;

    StopLoopAudioIfRunning();

    exit_code = system(cmd.c_str());
    if(exit_code != 0)
    {
        MSS_ROS_ERROR("The command \"%s\" returned error exit code %d",
                      cmd.c_str(), exit_code);
    }
}


void
Mpg123Player::PlayLoop(std::string audio_file_path)
{
    std::string cmd = GetLoopCmd(audio_file_path);

    try
    {
        loop_audio_cmd_.StopCmd();
        loop_audio_cmd_.StartCmd(cmd.c_str(), kLoopAudioTimeoutInMs);
    }
    catch(const AsyncCmdRunnerException& err)
    {
        MSS_ROS_ERROR("Couldn't play audio in a loop with command \"%s\"",
                      cmd.c_str());
    }
}


void
Mpg123Player::StopLoopAudioIfRunning(void)
{
    loop_audio_cmd_.StopCmd();
}


void
Mpg123Player::SetPercentageVolume(int percent_vol)
{
    if((kMinPercentageVol <= percent_vol) && (percent_vol <= kMaxPercentageVol))
    {
        percentage_volume_ = percent_vol;
        SetScaleFactorFromPercentVol(percent_vol);
    }
    else
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The provided percentage volume (0 to 100) is invalid"
                       << " " << percent_vol;

        throw std::invalid_argument(err_msg_stream.str());
    }
}


int
Mpg123Player::GetPercentageVolume(void)
{
    return percentage_volume_;
}


void
Mpg123Player::SetScaleFactorFromPercentVol(int percent_vol)
{
    const static double kOffset = std::log(kExpOffset);
    const static double kScaleVol = (std::log(kMaxScaleFactor + kExpOffset)
                                     - kOffset);

    double scaled_vol = ((kScaleVol * (double) percent_vol)
                         / ((double) kMaxPercentageVol));

    scale_factor_ = (int) (std::exp(scaled_vol + kOffset) - kExpOffset);
    scale_factor_ = ((scale_factor_ < kMinScaleFactor)
                     ? kMinScaleFactor : scale_factor_);
    scale_factor_ = ((scale_factor_ > kMaxScaleFactor)
                     ? kMaxScaleFactor : scale_factor_);
}


std::string
Mpg123Player::GetCmd(std::string audio_file_path)
{
    std::stringstream cmd_stream;

    cmd_stream << kMpg123Cmd << " " << kMpg123QuietOutput << " "
               << kMpg123ChangeScaleFactor << " "
               << scale_factor_ << " " << audio_file_path;

    return cmd_stream.str();
}


std::string
Mpg123Player::GetLoopCmd(std::string audio_file_path)
{
    std::stringstream cmd_stream;

    cmd_stream << kMpg123Cmd << " " << kMpg123InfiniteLoop << " "
               << kMpg123QuietOutput << " " << kMpg123ChangeScaleFactor << " "
               << scale_factor_ << " " << audio_file_path;

    return cmd_stream.str();
}
