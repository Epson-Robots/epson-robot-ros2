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


#include <thread>

#include "epson_robot_control/epson_robot_hardware_interface.hpp"

namespace epson_robot_control
{
	const rclcpp::Logger logger_ = rclcpp::get_logger("epson_robot_hardware_interface");

	EpsonRobotHwIf::EpsonRobotHwIf()
	{
		cmd_.assign(num_joints_, 0.0);
		cmd_.shrink_to_fit();
		pos_.assign(num_joints_, 0.0);
		pos_.shrink_to_fit();
		num_joints_=0;
		launch_file_pid_=0;
	}
	
	EpsonRobotHwIf::~EpsonRobotHwIf(){
		robot_driver_.reset();
	}
	
	CallbackReturn EpsonRobotHwIf::on_init(const hardware_interface::HardwareInfo& info)
	{
		RCLCPP_INFO(logger_, "on_init process starts");
		hw_info_ = info;
		
		if(hw_info_.joints.size() > 65535)
			{
				RCLCPP_ERROR(logger_, "The joint has exceeded its maximum limit");
				throw false;
			}
		num_joints_ = static_cast<unsigned short>(hw_info_.joints.size());
		int log_level = 10 * stoi(hw_info_.hardware_parameters["log_level"]);
		rcutils_ret_t util_ret;
		 util_ret=rcutils_logging_set_logger_level
			("epson_robot_hardware_interface", log_level);
		 (void) util_ret;
		 util_ret=rcutils_logging_set_logger_level
			("epson_robot_driver", log_level);

		launch_file_pid_ = getppid();

		for(unsigned short jnt=0;jnt<num_joints_;jnt++)
		{	
			if(hw_info_.joints[jnt].command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
			{
				RCLCPP_FATAL(logger_, "Initialization failed because command interfaces are not position interfaces");
				terminate_ros_driver();
				return CallbackReturn::ERROR;
			}

			if(hw_info_.joints[jnt].state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
			{
				RCLCPP_FATAL(logger_, "Initialization failed because state interfaces are not position interfaces");
				terminate_ros_driver();
				return CallbackReturn::ERROR;
			}
		}


		robot_driver_=make_shared<EpsonRobotDriver>();

		if(!robot_driver_->Init(hw_info_))
		{
			RCLCPP_FATAL(logger_, "on_init process failed,Init");
			terminate_ros_driver();
			return CallbackReturn::ERROR;
		}

		robot_driver_->Prepare_ServiceTopic();

		spin_node(robot_driver_);


		RCLCPP_INFO(logger_, "on_init process has completed");
		return CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface> EpsonRobotHwIf::export_state_interfaces()
	{
		RCLCPP_INFO(logger_, "export_state_interfaceses process starts");
		std::vector<hardware_interface::StateInterface> state_interfaces;
		for (unsigned short i = 0; i < num_joints_; i++) {
			state_interfaces.emplace_back
				(hardware_interface::StateInterface
				 (hw_info_.joints[i].name.c_str(),
				  hardware_interface::HW_IF_POSITION, &pos_[i]));
		}
		RCLCPP_INFO(logger_, "export_state_interfaceses process has completed");
		return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> EpsonRobotHwIf::export_command_interfaces()
	{
		RCLCPP_INFO(logger_, "export_command_interfaceses process starts");
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		for (unsigned short i = 0; i < num_joints_; i++) {
			command_interfaces.emplace_back
				(hardware_interface::CommandInterface
				 (hw_info_.joints[i].name.c_str(),
				  hardware_interface::HW_IF_POSITION, &cmd_[i]));
		}
		RCLCPP_INFO(logger_, "export_command_interfaceses process has completed");
		return command_interfaces;
	}
	
	CallbackReturn EpsonRobotHwIf::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(logger_, "on_configure process starts");
		
		if (!robot_driver_->Connect()) {
			RCLCPP_FATAL(logger_, "on_configure process failed,Connect");
			terminate_ros_driver();
			return CallbackReturn::ERROR;
		}

		short ret=robot_driver_->Prepare_RB_Activate();
		if (ret != CLIENT_SUCCESS)
		{
			RCLCPP_FATAL(logger_, "on_configure process failed,Prepare_RB_Activate");
			terminate_ros_driver();
			return CallbackReturn::ERROR;
		}
		
		
		
		RCLCPP_INFO(logger_, "on_configure process has completed");
		return CallbackReturn::SUCCESS;
	}
	
	CallbackReturn EpsonRobotHwIf::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(logger_, "on_activate process starts");

		short ret = robot_driver_->Robot_Activate(cmd_,pos_);
		if (ret==CLIENT_COM_ERR || ret==CLIENT_UNDEF_ERR)
		{
			RCLCPP_FATAL(logger_, "on_activate process failed");
			terminate_ros_driver();
			return CallbackReturn::ERROR;
		}
		if(ret != CLIENT_SUCCESS) RCLCPP_ERROR(logger_, "on_activate process failed");

		RCLCPP_INFO(logger_, "on_activate process has completed");
		return CallbackReturn::SUCCESS;
	}
	
	return_type EpsonRobotHwIf::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
	{

		bool ret=robot_driver_->Read_Proc(cmd_,pos_);
		if(!ret){
			// Kill parent process
			terminate_ros_driver();
			return return_type::ERROR;
		}
	
		return return_type::OK;
	}
	
	return_type EpsonRobotHwIf::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
	{


		bool ret=robot_driver_->Write_Proc(cmd_,pos_);
		
		if(!ret){
			RCLCPP_FATAL(logger_, "write process failed");
			// Kill parent process
			terminate_ros_driver();
			return return_type::ERROR;
		}
		return return_type::OK;
	}
	
	CallbackReturn EpsonRobotHwIf::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(logger_, "on_deactivate process starts");
		
		short ret=robot_driver_->Robot_Deactivate();
		if (ret==CLIENT_COM_ERR || ret==CLIENT_UNDEF_ERR)
		{
			RCLCPP_FATAL(logger_, "on_deactivate process failed");
			terminate_ros_driver();
			return CallbackReturn::ERROR;
		}
		
		RCLCPP_INFO(logger_, "on_deactivate process has completed");
		return CallbackReturn::SUCCESS;
	}

	CallbackReturn EpsonRobotHwIf::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(logger_, "on_cleanup process starts");
		
		robot_driver_->DisConnect();
		
		RCLCPP_INFO(logger_, "on_cleanup process has completed");
		return CallbackReturn::SUCCESS;
	}
	
	void EpsonRobotHwIf::spin_node(EpsonRobotDriver_Ptr robot_driver)
	{
		RCLCPP_INFO(logger_, "Starting \"epson_robot_control\" node...");
		std::thread robot_control_node_thread([robot_driver]() {
			while (rclcpp::ok()) {
				rclcpp::spin(robot_driver->get_node());
			}
			rclcpp::shutdown();
		});
		robot_control_node_thread.detach();
		RCLCPP_INFO(logger_, "Stopping \"epson_robot_control\" node");
	}

	void EpsonRobotHwIf::terminate_ros_driver(void)
	{
		RCLCPP_FATAL(logger_, "Terminate epson robot ROS2 driver. [%d]", launch_file_pid_);
		sleep(1); // Allow time to output error message to standard output before terminateing the process
		kill(launch_file_pid_, SIGINT);
		return;
	}
	
}// ending of namespace epson_robot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(epson_robot_control::EpsonRobotHwIf, hardware_interface::SystemInterface)
