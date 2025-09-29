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


#ifndef EPSON_ROBOT_DRIVER_HPP
#define EPSON_ROBOT_DRIVER_HPP

#include <memory>
#include <bitset>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "epson_robot_control/epson_robot_definitions.hpp"

#include "rtmc/rtmc_client.h"
#include "rtmc/rtmc_definition.h"

#include "epson_robot_msgs/msg/rc_status.hpp"
#include "epson_robot_msgs/msg/digital_output_common.hpp"
#include "epson_robot_msgs/msg/digital_input_common.hpp"

#include "epson_robot_msgs/srv/rb_check.hpp"
#include "epson_robot_msgs/srv/rb_check_result.hpp"
#include "epson_robot_msgs/srv/get_rb_model.hpp"
#include "epson_robot_msgs/srv/set_rtmc_send_format.hpp"
#include "epson_robot_msgs/srv/get_rtmc_send_format.hpp"
#include "epson_robot_msgs/srv/set_rtmc_recv_format.hpp"
#include "epson_robot_msgs/srv/get_rtmc_recv_format.hpp"
#include "epson_robot_msgs/srv/motor_on.hpp"
#include "epson_robot_msgs/srv/motor_off.hpp"
#include "epson_robot_msgs/srv/get_motor_status.hpp"
#include "epson_robot_msgs/srv/power_high.hpp"
#include "epson_robot_msgs/srv/power_low.hpp"
#include "epson_robot_msgs/srv/get_power_mode.hpp"
#include "epson_robot_msgs/srv/set_rtmc_mode_enable.hpp"
#include "epson_robot_msgs/srv/set_rtmc_mode_disable.hpp"
#include "epson_robot_msgs/srv/get_rtmc_mode.hpp"
#include "epson_robot_msgs/srv/get_current_ja.hpp"
#include "epson_robot_msgs/srv/reset.hpp"
#include "epson_robot_msgs/srv/set_digital_output_bit.hpp"
#include "epson_robot_msgs/srv/set_digital_output_byte.hpp"
#include "epson_robot_msgs/srv/get_digital_output_bit.hpp"
#include "epson_robot_msgs/srv/get_digital_output_byte.hpp"
#include "epson_robot_msgs/srv/get_digital_output_word.hpp"
#include "epson_robot_msgs/srv/get_digital_input_bit.hpp"
#include "epson_robot_msgs/srv/get_digital_input_byte.hpp"
#include "epson_robot_msgs/srv/get_digital_input_word.hpp"
#include "epson_robot_msgs/srv/terminate.hpp"
#include "epson_robot_msgs/srv/set_weight.hpp"
#include "epson_robot_msgs/srv/get_weight.hpp"
#include "epson_robot_msgs/srv/set_inertia.hpp"
#include "epson_robot_msgs/srv/get_inertia.hpp"
#include "epson_robot_msgs/srv/set_eccentricity.hpp"
#include "epson_robot_msgs/srv/get_eccentricity.hpp"
#include "epson_robot_msgs/srv/set_buffer_size.hpp"
#include "epson_robot_msgs/srv/get_buffer_size.hpp"

using namespace std;

namespace epson_robot_control
{
	class EpsonRobotDriver {
	public:
		EpsonRobotDriver(); 
		virtual ~EpsonRobotDriver();
		bool Init(const hardware_interface::HardwareInfo& hw_info);
		bool Connect();
		void Prepare_ServiceTopic();
		short Prepare_RB_Activate();
		void DisConnect();
		short Robot_Activate(vector<double>& pcmd_,vector<double>& ppos_);
		bool Read_Proc(vector<double>& pcmd_,vector<double>& ppos_);
		bool Write_Proc(const vector<double>& cmd_, vector<double>& ppos_);
		short Robot_Deactivate();
		rclcpp::Node::SharedPtr& get_node(void);

	private:
		void terminate(void);		
		void Set_SendRecvMode();
		void Update_StatusInfo(epson_rtmc_client::RCSTATUS temp_rc_status);
		short Wrap_Exec_RTMC(vector<double> cmd_,vector<double>& ppos_);
		short Wrap_Get_CurrentJA(vector<double>& pcmd_,vector<double>& ppos_);
		void OutputErrLog(short err,const string& clientapi);
		void meter2millimeter(double* p_scara_j3 );
		void millimeter2meter(double* p_scara_j3);

		void RBCheck_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::RBCheck::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::RBCheck::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::RBCheck>::SharedPtr RBCheck_service_;

		void RBCheckResult_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::RBCheckResult::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::RBCheckResult::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::RBCheckResult>::SharedPtr RBCheckResult_service_;

		void Get_RBModel_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRBModel::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetRBModel::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetRBModel>::SharedPtr Get_RBModel_service_;

		void Set_RTMCSendFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCSendFormat::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCSendFormat::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetRTMCSendFormat>::SharedPtr Set_RTMCSendFormat_service_;

		void Get_RTMCSendFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRTMCSendFormat::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetRTMCSendFormat::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetRTMCSendFormat>::SharedPtr Get_RTMCSendFormat_service_;

		void Set_RTMCRecvFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCRecvFormat::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCRecvFormat::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetRTMCRecvFormat>::SharedPtr Set_RTMCRecvFormat_service_;

		void Get_RTMCRecvFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRTMCRecvFormat::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetRTMCRecvFormat::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetRTMCRecvFormat>::SharedPtr Get_RTMCRecvFormat_service_;

		void MotorOn_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::MotorOn::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::MotorOn::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::MotorOn>::SharedPtr MotorOn_service_;

		void MotorOff_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::MotorOff::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::MotorOff::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::MotorOff>::SharedPtr MotorOff_service_;

		
		void Get_MotorStatus_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetMotorStatus::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetMotorStatus::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetMotorStatus>::SharedPtr Get_MotorStatus_service_;

		void PowerHigh_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::PowerHigh::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::PowerHigh::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::PowerHigh>::SharedPtr PowerHigh_service_;

		void PowerLow_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::PowerLow::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::PowerLow::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::PowerLow>::SharedPtr PowerLow_service_;

		void Get_PowerMode_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetPowerMode::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetPowerMode::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetPowerMode>::SharedPtr Get_PowerMode_service_;

		void Set_RTMCModeEnable_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeEnable::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeEnable::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetRTMCModeEnable>::SharedPtr Set_RTMCModeEnable_service_;

		void Set_RTMCModeDisable_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeDisable::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeDisable::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetRTMCModeDisable>::SharedPtr Set_RTMCModeDisable_service_;

		void Get_RTMCMode_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRTMCMode::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetRTMCMode::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetRTMCMode>::SharedPtr Get_RTMCMode_service_;

		void Get_CurrentJA_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetCurrentJA::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetCurrentJA::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetCurrentJA>::SharedPtr Get_CurrentJA_service_;

		void Reset_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::Reset::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::Reset::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::Reset>::SharedPtr Reset_service_;


		void Set_DigitalOutput_Bit_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputBit::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputBit::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetDigitalOutputBit>::SharedPtr Set_DigitalOutput_Bit_service_;

		void Set_DigitalOutput_Byte_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputByte::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputByte::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetDigitalOutputByte>::SharedPtr Set_DigitalOutput_Byte_service_;
		
		void Get_DigitalOutput_Bit_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputBit::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputBit::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetDigitalOutputBit>::SharedPtr Get_DigitalOutput_Bit_service_;

		void Get_DigitalOutput_Byte_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputByte::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputByte::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetDigitalOutputByte>::SharedPtr Get_DigitalOutput_Byte_service_;
				
		void Get_DigitalOutput_Word_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputWord::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputWord::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetDigitalOutputWord>::SharedPtr Get_DigitalOutput_Word_service_;
		

		void Get_DigitalInput_Bit_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputBit::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputBit::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetDigitalInputBit>::SharedPtr Get_DigitalInput_Bit_service_;

		void Get_DigitalInput_Byte_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputByte::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputByte::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetDigitalInputByte>::SharedPtr Get_DigitalInput_Byte_service_;

		void Get_DigitalInput_Word_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputWord::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputWord::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetDigitalInputWord>::SharedPtr Get_DigitalInput_Word_service_;

		void Terminate_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::Terminate::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::Terminate::Response> /*response*/);
		rclcpp::Service<epson_robot_msgs::srv::Terminate>::SharedPtr Terminate_service_;

		void Set_Weight_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetWeight::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetWeight::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetWeight>::SharedPtr Set_Weight_service_;

		void Get_Weight_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetWeight::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetWeight::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetWeight>::SharedPtr Get_Weight_service_;

		void Set_Inertia_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetInertia::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetInertia::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetInertia>::SharedPtr Set_Inertia_service_;

		void Get_Inertia_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetInertia::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetInertia::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetInertia>::SharedPtr Get_Inertia_service_;

		void Set_Eccentricity_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetEccentricity::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetEccentricity::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetEccentricity>::SharedPtr Set_Eccentricity_service_;

		void Get_Eccentricity_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetEccentricity::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetEccentricity::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetEccentricity>::SharedPtr Get_Eccentricity_service_;

		void Set_BufferSize_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetBufferSize::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetBufferSize::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::SetBufferSize>::SharedPtr Set_BufferSize_service_;

		void Get_BufferSize_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetBufferSize::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetBufferSize::Response> response);
		rclcpp::Service<epson_robot_msgs::srv::GetBufferSize>::SharedPtr Get_BufferSize_service_;


		rclcpp::Publisher<epson_robot_msgs::msg::RCStatus>::SharedPtr rc_state_publisher_;
		void rc_state_publisher_func(void);

		rclcpp::Subscription<epson_robot_msgs::msg::DigitalOutputCommon>::SharedPtr digital_output_common_subscriber_;
		void digital_output_common_subscriber_func(const epson_robot_msgs::msg::DigitalOutputCommon do_msg);

		rclcpp::Publisher<epson_robot_msgs::msg::DigitalInputCommon>::SharedPtr digital_input_common_publisher_;
		void digital_input_common_publisher_func(void);

				
		hardware_interface::HardwareInfo initial_hw_info_;
		epson_rtmc_client::RCSTATUS rc_status;
		unsigned short num_joints_;
		epson_rtmc_client::COMPROPERTY com_property;
		epson_rtmc_client::COMPROPERTYOPT com_property_option;
		unsigned short rtmc_send_format;
		unsigned short rtmc_recv_format;
		unsigned long sendrecv_mode;
		string rb_model;
		epson_rtmc_client::SECPROPERTY sec_property;
		string password;
		string epson_service_topic_name;
		unsigned short bufsize;
		struct weight{
			bool weight_default;
			double weight_val;
		} weight;

		struct inertia{
			bool inertia_default;
			double inertia_val;
		} inertia;
		struct eccentricity{
			bool eccentricity_default;
			unsigned short eccentricity_val;
		} eccentricity;

		rclcpp::Node::SharedPtr node_;
		unsigned short cmddo;
		unsigned int curdi;
		unsigned int get_di;
		epson_rtmc_client::RTMCClient* rtmc_client_;
		bool robot_control_flg;
		bool ExecTerminateProcess;
		bool NeedOutputErrLog_forGetCurrentJA;
	};
	typedef std::shared_ptr<EpsonRobotDriver> EpsonRobotDriver_Ptr;
} // namespace epson_robot_control
#endif // EPSON_ROBOT_DRIVER_HPP
