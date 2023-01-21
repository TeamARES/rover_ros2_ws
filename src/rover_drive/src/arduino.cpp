#include "rover_drive/arduino.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

void Arduino::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial_connection_.setPort(serial_device);
    serial_connection_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_connection_.setTimeout(tt);
    serial_connection_.open();
}

void Arduino::sendEmptyMessage()
{
    std::string response = Arduino::sendMessage("\r");
}

void Arduino::readEncoderValues(int &forward_left_encoder_value, int &forward_right_encoder_value, int &rear_left_encoder_value, int &rear_right_encoder_value){
    std::string response = sendMessage("e\r");
    // response format: "[fl] [fr] [rl] [rr]"
    std::string delimiter = " ";
    size_t delimiter_position_1 = response.find(delimiter);
    std::string token_1 = response.substr(0, delimiter_position_1);
    
    size_t delimiter_position_2 = response.substr(delimiter_position_1 + delimiter.length()).find(delimiter);
    std::string token_2 = response.substr(delimiter_position_1 + delimiter.length(), delimiter_position_2);

    size_t delimiter_position_3 = response.substr(delimiter_position_2 + delimiter.length()).find(delimiter);
    std::string token_3 = response.substr(delimiter_position_2 + delimiter.length(), delimiter_position_3);

    std::string token_4 = response.substr(delimiter_position_3 + delimiter.length());

    forward_left_encoder_value = std::atoi(token_1.c_str());
    forward_right_encoder_value = std::atoi(token_2.c_str());
    rear_left_encoder_value = std::atoi(token_3.c_str());
    rear_right_encoder_value = std::atoi(token_4.c_str());
}

void Arduino::writeMotorValues(int forward_left_motor_value, int forward_right_motor_value, int rear_left_motor_value, int rear_right_motor_value){
    std::stringstream ss;

    //format: "[fl] [fr] [rl] [rr]"
    ss << "m " << forward_left_motor_value << " " << forward_right_motor_value << " " << rear_left_motor_value << " " << rear_right_motor_value << "\r";
    Arduino::sendMessage(ss.str());
}

void Arduino::writePIDValues(float k_p, float k_i, float k_d, float k_o){
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_i << ":" << k_d << ":" << k_o << "\r";
    Arduino::sendMessage(ss.str());
}

std::string Arduino::sendMessage(const std::string &message, bool print_output = false){
    serial_connection_.write(message);
    std::string response = serial_connection_.readline();

    if(print_output){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("PropulsionHardware"), "Sent: " << message);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("PropulsionHardware"), "Received: " << message);
    }
    return response;
}