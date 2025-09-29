/*
  Copyright 2025 Seiko Epson Corporation

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/


#ifndef EPSON_ROBOT_HARDWARE_INTERFACE_HPP
#define EPSON_ROBOT_HARDWARE_INTERFACE_HPP

#include "epson_robot_control/epson_robot_driver.hpp"

namespace epson_robot_control
{
	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
	using hardware_interface::return_type;
	
	class EpsonRobotHwIf : public hardware_interface::SystemInterface
	{
	public:
		EpsonRobotHwIf();

		~EpsonRobotHwIf();
		
		CallbackReturn on_init
		(const hardware_interface::HardwareInfo& info) override;

		std::vector<hardware_interface::StateInterface>
		export_state_interfaces() override;

		std::vector<hardware_interface::CommandInterface>
		export_command_interfaces() override;

		CallbackReturn on_configure
		(const rclcpp_lifecycle::State & /*previous_state*/) override;

		CallbackReturn on_activate
		(const rclcpp_lifecycle::State & /*previous_state*/) override;

		return_type read
		(const rclcpp::Time& /*time*/,
		 const rclcpp::Duration& /*period*/) override;

		return_type write
		(const rclcpp::Time& /*time*/,
		 const rclcpp::Duration& /*period*/) override;

		CallbackReturn on_deactivate
		(const rclcpp_lifecycle::State & /*previous_state*/) override;

		CallbackReturn on_cleanup
		(const rclcpp_lifecycle::State & /*previous_state*/) override;
		
	private:
		// Original function
		void spin_node(EpsonRobotDriver_Ptr robot_driver);
		void terminate_ros_driver(void);
		
		EpsonRobotDriver_Ptr robot_driver_;
		hardware_interface::HardwareInfo hw_info_;
		std::vector<double> cmd_;
		std::vector<double> pos_;
		unsigned short num_joints_;
		int launch_file_pid_;

	};
}// ending of namespace epson_robot_control
#endif // EPSON_ROBOT_HARDWARE_INTERFACE_HPP
