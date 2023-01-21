#ifndef ROVER_DRIVE__ARDUINO_HPP_
#define ROVER_DRIVE__ARDUINO_HPP_

#include <serial/serial.h>
#include <string>

class Arduino
{
public:
    Arduino() {}

    Arduino(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms) : serial_connection_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms)) {}

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void sendEmptyMessage();
    void readEncoderValues(int &forward_left_encoder_value, int &forward_right_encoder_value, int &rear_left_encoder_value, int &rear_right_encoder_value);
    void writeMotorValues(int forward_left_motor_value, int forward_right_motor_value, int rear_left_motor_value, int rear_right_motor_value);
    void writePIDValues(float k_p, float k_i, float k_d, float k_o);

    bool connected() const { return serial_connection_.isOpen(); }

    std::string sendMessage(const std::string &message, bool print_output = false);

private:
    serial::Serial serial_connection_;
};

#endif