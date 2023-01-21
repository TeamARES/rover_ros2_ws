#ifndef ROVER_DRIVE__CONFIG_HPP_
#define ROVER_DRIVE__CONFIG_HPP_

#include <string>

struct Config{
    std::string forward_left_wheel_name = "forward_left_wheel";
    std::string forward_right_wheel_name = "forward_right_wheel";
    std::string rear_left_wheel_name = "rear_left_wheel";
    std::string rear_right_wheel_name = "rear_right_wheel";
    float loop_rate = 30;
    std::string device = "/dev/ttyUSB0";
    int baud_rate = 57600;
    int timeout = 1000;
    int forward_left_enc_counts_per_rev = 1920;
    int forward_right_enc_counts_per_rev = 1920;
    int rear_left_enc_counts_per_rev = 1920;
    int rear_right_enc_counts_per_rev = 1920;
};

#endif
