#include "rover_drive/propulsion_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_drive
{
    hardware_interface::return_type PropulsionInterfaceHardware::configure(const hardware_interface::HardwareInfo &info)
    {
        if (configure_default(info) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("PropulsionHardware"), "Configuring...");

        time_ = std::chrono::system_clock::now();


        config_.forward_left_wheel_name = info_.hardware_parameters["forward_left_wheel_name"];
        config_.forward_right_wheel_name = info_.hardware_parameters["forward_right_wheel_name"];
        config_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
        config_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        config_.device = info_.hardware_parameters["device"];
        config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        config_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
        config_.forward_left_enc_counts_per_rev = std::stoi(info_.hardware_parameters["forward_left_enc_counts_per_rev"]);
        config_.forward_right_enc_counts_per_rev = std::stoi(info_.hardware_parameters["forward_right_enc_counts_per_rev"]);
        config_.rear_left_enc_counts_per_rev = std::stoi(info_.hardware_parameters["rear_left_enc_counts_per_rev"]);
        config_.rear_right_enc_counts_per_rev = std::stoi(info_.hardware_parameters["rear_right_enc_counts_per_rev"]);

        forward_left_wheel_.setup(config_.forward_left_wheel_name, config_.forward_left_enc_counts_per_rev);
        forward_right_wheel_.setup(config_.forward_right_wheel_name, config_.forward_right_enc_counts_per_rev);
        rear_left_wheel_.setup(config_.rear_left_wheel_name, config_.rear_left_enc_counts_per_rev);
        rear_right_wheel_.setup(config_.rear_right_wheel_name, config_.rear_right_enc_counts_per_rev);

        arduino_.setup(config_.device, config_.baud_rate, config_.timeout);

        RCLCPP_INFO(rclcpp::get_logger("PropulsionHardware"), "Finished Configuration.");

        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> PropulsionInterfaceHardware::export_state_interfaces()
    {
        // Set up a position and velocity state interface for each wheel
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(forward_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &forward_left_wheel_.vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(forward_left_wheel_.name, hardware_interface::HW_IF_POSITION, &forward_left_wheel_.pos));

        state_interfaces.emplace_back(hardware_interface::StateInterface(forward_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &forward_right_wheel_.vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(forward_right_wheel_.name, hardware_interface::HW_IF_POSITION, &forward_right_wheel_.pos));

        state_interfaces.emplace_back(hardware_interface::StateInterface(rear_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_.vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(rear_left_wheel_.name, hardware_interface::HW_IF_POSITION, &rear_left_wheel_.pos));

        state_interfaces.emplace_back(hardware_interface::StateInterface(rear_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_.vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(rear_right_wheel_.name, hardware_interface::HW_IF_POSITION, &rear_right_wheel_.pos));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> PropulsionInterfaceHardware::export_command_interfaces()
    {
        // Set up a velocity command interface for each wheel

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(forward_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &forward_left_wheel_.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(forward_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &forward_right_wheel_.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(rear_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(rear_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_.cmd));

        return command_interfaces;
    }

    hardware_interface::return_type PropulsionInterfaceHardware::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("PropulsionHardware"), "Starting Controller...");

        arduino_.sendEmptyMessage();

        // TODO:SET PID VALUES
        // arduino_.writePIDValues(30, 20, 0, 100);

        status_ = hardware_interface::status::STARTED;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropulsionInterfaceHardware::stop()
    {
        RCLCPP_INFO(rclcpp::get_logger("PropulsionHardware"), "Stopping Controller...");

        status_ = hardware_interface::status::STOPPED;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropulsionInterfaceHardware::read()
    {
        auto new_time_ = std::chrono::system_clock::now();
        std::chrono::duration<double> difference = new_time_ - time_;
        double delta_sec = difference.count();
        time_ = new_time_;

        if (!arduino_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        arduino_.readEncoderValues(forward_left_wheel_.enc, forward_right_wheel_.enc, rear_left_wheel_.enc, rear_right_wheel_.enc);

        double previous_pos = forward_left_wheel_.pos;
        forward_left_wheel_.pos = forward_left_wheel_.calcEncAngle();
        forward_left_wheel_.vel = (forward_left_wheel_.pos - previous_pos) / delta_sec;

        previous_pos = forward_right_wheel_.pos;
        forward_right_wheel_.pos = forward_right_wheel_.calcEncAngle();
        forward_right_wheel_.vel = (forward_right_wheel_.pos - previous_pos) / delta_sec;

        previous_pos = rear_left_wheel_.pos;
        rear_left_wheel_.pos = rear_left_wheel_.calcEncAngle();
        rear_left_wheel_.vel = (rear_left_wheel_.pos - previous_pos) / delta_sec;

        previous_pos = rear_right_wheel_.pos;
        rear_right_wheel_.pos = rear_right_wheel_.calcEncAngle();
        rear_right_wheel_.vel = (rear_right_wheel_.pos - previous_pos) / delta_sec;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropulsionInterfaceHardware::write()
    {
        if(!arduino_.connected()){
            return hardware_interface::return_type::ERROR;
        }

        arduino_.writeMotorValues(forward_left_wheel_.cmd / forward_left_wheel_.rads_per_count / config_.loop_rate, forward_right_wheel_.cmd / forward_right_wheel_.rads_per_count / config_.loop_rate, rear_left_wheel_.cmd / rear_left_wheel_.rads_per_count / config_.loop_rate, rear_right_wheel_.cmd / rear_right_wheel_.rads_per_count / config_.loop_rate);

        return hardware_interface::return_type::OK;
    }

#include "pluginlib/class_list_macros.hpp"

    PLUGINLIB_EXPORT_CLASS(
        rover_drive::PropulsionInterfaceHardware,
        hardware_interface::SystemInterface)
}