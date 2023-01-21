#ifndef ROVER_DRIVE__PROPULSION_INTERFACE_HPP_
#define ROVER_DRIVE__PROPULSION_INTERFACE_HPP_

#include <memory>
#include <cstring>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"

#include "config.hpp"
#include "wheel.hpp"
#include "arduino.hpp"

namespace rover_drive
{
    class PropulsionInterfaceHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropulsionInterfaceHardware)

        hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type start() override;
         
        hardware_interface::return_type stop() override;

        hardware_interface::return_type read() override;

        hardware_interface::return_type write() override;

    private:
        Config config_;
        Arduino arduino_;
        Wheel forward_left_wheel_;
        Wheel forward_right_wheel_;
        Wheel rear_left_wheel_;
        Wheel rear_right_wheel_;
        std::chrono::time_point<std::chrono::system_clock> time_;
    };
}

#endif