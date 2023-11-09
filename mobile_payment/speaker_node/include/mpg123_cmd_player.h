#pragma once

#include <string>

#include "mss_utils/async_cmd_runner.h"


class Mpg123Player
{
    public:
        void Play(std::string audio_file_path);

        void PlayLoop(std::string audio_file_path);

        void StopLoopAudioIfRunning(void);

        void SetPercentageVolume(int percent_vol);

        int GetPercentageVolume(void);

    private:
        const int kLoopAudioTimeoutInMs = 15000;

        const char* kMpg123Cmd = "mpg123";

        const char* kMpg123QuietOutput = "-q";

        const char* kMpg123ChangeScaleFactor = "-f";

        const char* kMpg123InfiniteLoop = "--loop -1";

        const int kMinPercentageVol = 0;

        const int kMaxPercentageVol = 100;

        const int kMinScaleFactor = 0;

        const int kMaxScaleFactor = 32768;

        const double kExpOffset = 2000.0;

        int percentage_volume_ = kMaxPercentageVol;

        int scale_factor_ = kMaxScaleFactor;

        AsyncCmdRunner loop_audio_cmd_;

        void SetScaleFactorFromPercentVol(int percent_vol);

        std::string GetCmd(std::string audio_file_path);

        std::string GetLoopCmd(std::string audio_file_path);
};
