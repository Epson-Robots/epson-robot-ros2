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

#include <memory>
#include "epson_robot_control/epson_robot_driver.hpp"

namespace epson_robot_control{
	const rclcpp::Logger logger_ = rclcpp::get_logger("epson_robot_driver");

	EpsonRobotDriver::EpsonRobotDriver() : rc_status{0,0,0,0,0,{0,0,0,""},{0,0,0,""}}, com_property{"",0},com_property_option{"",0,0},sec_property{false,"","","","",""}
	{

		rtmc_send_format=0;
		rtmc_recv_format=0;
		bufsize=0;
		weight.weight_default=true;
		weight.weight_val=0.0;
		inertia.inertia_default=true;
		inertia.inertia_val=0.0;
		eccentricity.eccentricity_default=true;
		eccentricity.eccentricity_val=0.0;
		rtmc_client_=nullptr;

		num_joints_=0;
	
		sendrecv_mode=0;
		cmddo=0;
		curdi=0;
		get_di=0;
		robot_control_flg=false;
		ExecTerminateProcess=false;
		NeedOutputErrLog_forGetCurrentJA=true;

	}
	
	EpsonRobotDriver::~EpsonRobotDriver(){
		RCLCPP_FATAL(logger_, "Destractor called.");

		terminate();
	}

	//Retrieve the arguments of the launch file.
	bool EpsonRobotDriver::Init(const hardware_interface::HardwareInfo& hw_info)
	{
		initial_hw_info_=hw_info;
		
		string temp_str="";
		try{
			int tempi=0;

			if(initial_hw_info_.joints.size()> 65535)
			{
				RCLCPP_ERROR(logger_, "The joint has exceeded its maximum limit");
				throw false;
			}
			num_joints_ = static_cast<unsigned short>(initial_hw_info_.joints.size());

			com_property.rc_ip_address=initial_hw_info_.hardware_parameters["controller_ip"];

			tempi=stoi(initial_hw_info_.hardware_parameters["controller_port"]);
			if(tempi < 0 || tempi> 65535)
			{
				RCLCPP_ERROR(logger_, "controller_port is out of range");
				throw false;
			}
			com_property.rc_builtinmsg_port=static_cast<unsigned short>(tempi);

			temp_str=initial_hw_info_.hardware_parameters["security"];
			if(temp_str == "true" ||
			   temp_str == "True" ||
			   temp_str == "TRUE")
			{
				sec_property.security_communication_flag=true;
			}else{
				sec_property.security_communication_flag=false;
			}
			temp_str.clear();

			
			com_property_option.client_ip_address=initial_hw_info_.hardware_parameters["client_ip"];
			tempi=stoi(initial_hw_info_.hardware_parameters["client_builtinmsg_port"]);
			if(tempi < 0 || tempi> 65535)
			{
				RCLCPP_ERROR(logger_, "client_builtinmsg_port is out of range");
				throw false;
			}
			com_property_option.client_builtinmsg_port=static_cast<unsigned short>(tempi);


			tempi=stoi(initial_hw_info_.hardware_parameters["client_userdata_port"]);
			if(tempi < 0 || tempi> 65535)
			{
				RCLCPP_ERROR(logger_, "client_userdata_port is out of range");
				throw false;
				
			}
			com_property_option.client_userdata_port=static_cast<unsigned short>(tempi);

			tempi=stoi(initial_hw_info_.hardware_parameters["send_format"]);
			if(tempi < 0 || tempi> 65535)
			{
				RCLCPP_ERROR(logger_, "send_format is out of range");
				throw false;
			}
			rtmc_send_format=static_cast<unsigned short>(tempi);

			tempi=stoi(initial_hw_info_.hardware_parameters["recv_format"]);
			if(tempi < 0 || tempi> 65535)
			{
				RCLCPP_ERROR(logger_, "recv_format is out of range");
				throw false;
			}
			rtmc_recv_format=static_cast<unsigned short>(tempi);

			rb_model=initial_hw_info_.hardware_parameters["rb_model"];

			temp_str=initial_hw_info_.hardware_parameters["weight"];
			if(temp_str=="Default"){
				weight.weight_default=true;
			}else{
				weight.weight_default=false;
				weight.weight_val=stod(initial_hw_info_.hardware_parameters["weight"]);
			}
			temp_str.clear();

			temp_str=initial_hw_info_.hardware_parameters["inertia"];
			if(temp_str=="Default"){
				inertia.inertia_default=true;
			}else{
				inertia.inertia_default=false;
				inertia.inertia_val=stod(initial_hw_info_.hardware_parameters["inertia"]);
			}
			temp_str.clear();

			temp_str=initial_hw_info_.hardware_parameters["eccentricity"];
			if(temp_str=="Default"){
				eccentricity.eccentricity_default=true;
			}else{
				eccentricity.eccentricity_default=false;
				tempi=stoi(initial_hw_info_.hardware_parameters["eccentricity"]);
				if(tempi < 0 || tempi> 65535)
				{
				RCLCPP_ERROR(logger_, "eccentricity is out of range");
				throw false;
				}
				eccentricity.eccentricity_val=static_cast<unsigned short>(tempi);
			}
			temp_str.clear();

			sec_property.ca_cert_filepath=initial_hw_info_.hardware_parameters["ca_cert"];
			sec_property.client_cert_filepath=initial_hw_info_.hardware_parameters["client_cert"];
			sec_property.client_private_key=initial_hw_info_.hardware_parameters["key"];
			sec_property.client_governance_filepath=initial_hw_info_.hardware_parameters["governance"];
			sec_property.client_permissions_filepath=initial_hw_info_.hardware_parameters["permissions"];

			password=initial_hw_info_.hardware_parameters["password"];
			
			tempi=stoi(initial_hw_info_.hardware_parameters["buffer_size"]);
			if(tempi <= 0 || tempi> 65535)
			{
				RCLCPP_ERROR(logger_, "buffer_size is out of range");
				throw false;
			}
			bufsize=static_cast<unsigned short>(tempi);
			
			epson_service_topic_name=initial_hw_info_.hardware_parameters["namespace"];

			return true;
		}
		catch(const std::invalid_argument& err)
		{
			RCLCPP_ERROR(logger_, "Invalid argument");
			return false;
		}
		catch(const std::out_of_range& err)
		{
			RCLCPP_ERROR(logger_, "Data type overflow");
			return false;
		}
		catch(bool err){return err;}


	}

	bool EpsonRobotDriver::Connect()
	{
		rtmc_client_ =nullptr;
		rtmc_client_ = new epson_rtmc_client::RTMCClient;
		short ret=0;

		//Apply the communication settings for Real Time Motion Control provided as arguments.
		ret=rtmc_client_->Set_ComProperty(com_property,com_property_option);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_ComProperty");
			return false;
		}

		//Apply the secure communication settings for Real Time Motion Control provided as arguments.
		ret=rtmc_client_->Set_SecProperty(sec_property);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_SecProperty");
			return false;
		}

		//Establish communication with the controller.
		ret=rtmc_client_->RTMC_Connect(password);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"RTMC_Connect");
			return false;
		}

		return true;	
		
	}

	void EpsonRobotDriver::DisConnect()
	{

		short ret=0;

		//Terminate communication with the controller.
		ret=rtmc_client_->RTMC_DisConnect();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"RTMC_DisConnect");
			if(ret==CLIENT_ERR) 
			{
			 RCLCPP_FATAL(logger_, "The Connection to epson robot controller was forcibly terminated");
			}
		}
	
		
	}

	short EpsonRobotDriver::Robot_Activate(vector<double>& pcmd_, vector<double>& ppos_)
	{
		short ret=0;
		epson_rtmc_client::RCSTATUS temp_rc_status={0,0,0,0,0,{0,0,0,""},{0,0,0,""}};

		Set_SendRecvMode();

		//Reset the controller error, emergency stop state, and safety door open state.
		ret=rtmc_client_->Reset();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Reset");
			return ret;
		}

		//Retrieve the current joint angles of the robot
		ret=Wrap_Get_CurrentJA(pcmd_,ppos_);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Get_CurrentJA");
			return ret;
		}

		//Retrieve the current status of the robot controller
		ret=rtmc_client_->Get_RCStatus(temp_rc_status);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Get_RCStatus");
			return ret;
		}
		Update_StatusInfo(temp_rc_status);

		//Energize the motor.
		ret=rtmc_client_->MotorOn();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"MotorOn");
			return ret;
		}

		//Set the robot's power mode to High
		ret=rtmc_client_->PowerHigh();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"PowerHigh");
			return ret;
		}

		//Enable Real Time Motion Control mode
		ret=rtmc_client_->Set_RTMCModeEnable();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_RTMCModeEnable");
			return ret;
		}

		robot_control_flg=true;


		

	return CLIENT_SUCCESS;	
		
	}

	
	short EpsonRobotDriver::Robot_Deactivate()
	{
		
		//Disable Real Time Motion Control mode
		short ret=rtmc_client_->Set_RTMCModeDisable();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_RTMCModeDisable");
			return ret;
		}

		//De-energize the motor
		ret=rtmc_client_->MotorOff();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"MotorOff");
			return ret;
		}

		return CLIENT_SUCCESS;
			
	}

	bool EpsonRobotDriver::Read_Proc(vector<double>& pcmd_,vector<double>& ppos_)
	{
		if(!robot_control_flg)
		{

			//Retrieve the current status of the robot controller
			short ret=Wrap_Get_CurrentJA(pcmd_,ppos_);
			if(ret != CLIENT_SUCCESS)
			{
				if(NeedOutputErrLog_forGetCurrentJA) OutputErrLog(ret,"Get_CurrentJA");
				if(ret != CLIENT_RTMC_MODE_ERR) return false;
				NeedOutputErrLog_forGetCurrentJA=false;
			}
			
		}

		//Publish the status of standard I/O inputs to the user
		if(rtmc_recv_format == RTMC_RECV_FORMAT_RB_DI) digital_input_common_publisher_func();
		if(ExecTerminateProcess) return false;

		return true;	
		
	}

	bool EpsonRobotDriver::Write_Proc(const vector<double>& cmd_, vector<double>& ppos_)
	{
		short ret=0;
		epson_rtmc_client::RCSTATUS temp_rc_status={0,0,0,0,0,{0,0,0,""},{0,0,0,""}};

		if(robot_control_flg)
		{	
			//Send motion command values to each joint and retrieve the current joint angles of the robot.
			ret=Wrap_Exec_RTMC(cmd_, ppos_);
			if(ret != CLIENT_SUCCESS)
			{
				OutputErrLog(ret,"Exec_RTMC");
				robot_control_flg=false;

				if(ret == CLIENT_COM_ERR || ret == CLIENT_UNDEF_ERR ) return false;

			}
			
		}

		//Retrieve the current status of the robot controller
		ret=rtmc_client_->Get_RCStatus(temp_rc_status);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Get_RCStatus");
			return false;
		}
		//Publish the controller's status to the user.
		Update_StatusInfo(temp_rc_status);

		return true;	
		
	}

	void EpsonRobotDriver::rc_state_publisher_func()
	{
		auto msg_rc_status=epson_robot_msgs::msg::RCStatus();

		msg_rc_status.safeguard=rc_status.safeGuard;
		msg_rc_status.estop=rc_status.eStop;
		msg_rc_status.operation_mode=rc_status.operation_mode;
		msg_rc_status.err_num=rc_status.err_num;
		msg_rc_status.err_add_info1=rc_status.err_info.add_info1;
		msg_rc_status.err_add_info2=rc_status.err_info.add_info2;
		msg_rc_status.err_jnt=rc_status.err_info.jnt;
		msg_rc_status.err_msg=rc_status.err_info.msg;
		msg_rc_status.wrn_num=rc_status.wrn_num;
		msg_rc_status.wrn_add_info1=rc_status.wrn_info.add_info1;
		msg_rc_status.wrn_add_info2=rc_status.wrn_info.add_info2;
		msg_rc_status.wrn_jnt=rc_status.wrn_info.jnt;
		msg_rc_status.wrn_msg=rc_status.wrn_info.msg;

		rc_state_publisher_->publish(msg_rc_status);
			
	}

	void EpsonRobotDriver::digital_input_common_publisher_func()
	{

		auto msg_digital_input=epson_robot_msgs::msg::DigitalInputCommon();
		unsigned short low_worddata=0;
		unsigned short high_worddata=0;
		
		if(!robot_control_flg){

			rtmc_client_->Get_DigitalInput_Word(0,&low_worddata);
			
			rtmc_client_->Get_DigitalInput_Word(1,&high_worddata);
				
			bitset<16> low_worddata_bit(low_worddata);
			bitset<16> high_worddata_bit(high_worddata);
			bitset<32> worddata_bit(high_worddata_bit.to_string()+low_worddata_bit.to_string());

			get_di=static_cast<unsigned int>(worddata_bit.to_ulong());
		}

		if(curdi != get_di)
		{
			curdi=get_di;
			msg_digital_input.worddata=curdi;
			digital_input_common_publisher_->publish(msg_digital_input);
		}	
	}

	void EpsonRobotDriver::digital_output_common_subscriber_func(const epson_robot_msgs::msg::DigitalOutputCommon do_msg)
	{
		if(rtmc_send_format == RTMC_SEND_FORMAT_RB_DO)
		{
			if(!robot_control_flg){

				//Control standard I/O output using a 2-byte specification.
				short ret=rtmc_client_->Set_DigitalOutput_Word(do_msg.worddata);
				if(ret != CLIENT_SUCCESS)
				{
					OutputErrLog(ret,"Set_DigitalOutput_Word");
				}


			}else{
				//The standard I/O output is sent by the Exec_RTMC function.
				cmddo=do_msg.worddata;
			}
		}
	}


	void EpsonRobotDriver::Update_StatusInfo(epson_rtmc_client::RCSTATUS temp_rc_status)
	{
		bool pub_rcstatus=false;
		if(rc_status.safeGuard != temp_rc_status.safeGuard) 
		{
		   rc_status.safeGuard = temp_rc_status.safeGuard;

		   if(rc_status.safeGuard==RC_SG_OPEN) RCLCPP_WARN(logger_, "Safe Gurad is in an OPEN state");
		   if(rc_status.safeGuard==RC_SG_CLOSE) RCLCPP_INFO(logger_, "Safe Gurad is in an CLOSED state");
		   pub_rcstatus=true;

		}
		if(rc_status.eStop != temp_rc_status.eStop) 
		{
			rc_status.eStop = temp_rc_status.eStop;

			if(rc_status.eStop==RC_ESTOP_ON) RCLCPP_WARN(logger_, "Epson robot controller is in an emergency stop state");
		    if(rc_status.eStop==RC_ESTOP_OFF) RCLCPP_INFO(logger_, "The emergency stop state of epson robot controller has been released");
			pub_rcstatus=true;
		}
		if(rc_status.operation_mode != temp_rc_status.operation_mode) 
		{
			rc_status.operation_mode = temp_rc_status.operation_mode;

			if(rc_status.operation_mode == RC_OPERATION_MODE_Auto) RCLCPP_WARN(logger_, "Epson robot controller's operation mode is set to Auto mode");
			if(rc_status.operation_mode == RC_OPERATION_MODE_Program) RCLCPP_WARN(logger_, "Epson robot controller's operation mode is set to Program mode");
			if(rc_status.operation_mode == RC_OPERATION_MODE_Teach) RCLCPP_WARN(logger_, "Epson robot controller's operation mode is set to Teach mode");
			pub_rcstatus=true;
		}

		if(rc_status.err_num != temp_rc_status.err_num) 
		{
			rc_status.err_num = temp_rc_status.err_num;
			rc_status.err_info.add_info1=temp_rc_status.err_info.add_info1;
			rc_status.err_info.add_info2=temp_rc_status.err_info.add_info2;
			rc_status.err_info.jnt=temp_rc_status.err_info.jnt;
			rc_status.err_info.msg=temp_rc_status.err_info.msg;

			if(rc_status.err_num >0) RCLCPP_ERROR(logger_, "%s,error number [%u],additional information 1 [%u],additional information2[%u],axis[%u]",
													rc_status.err_info.msg.c_str(),rc_status.err_num,rc_status.err_info.add_info1,rc_status.err_info.add_info2,rc_status.err_info.jnt);
			if(rc_status.err_num==RC_NONERR) RCLCPP_INFO(logger_, "The error state has been cleared");

			pub_rcstatus=true;
		}

		if(rc_status.wrn_num != temp_rc_status.wrn_num) 
		{
			rc_status.wrn_num = temp_rc_status.wrn_num;
			rc_status.wrn_info.add_info1=temp_rc_status.wrn_info.add_info1;
			rc_status.wrn_info.add_info2=temp_rc_status.wrn_info.add_info2;
			rc_status.wrn_info.jnt=temp_rc_status.wrn_info.jnt;
			rc_status.wrn_info.msg=temp_rc_status.wrn_info.msg;

			if(rc_status.wrn_num >0) RCLCPP_WARN(logger_, "%s,warning number [%u],additional information 1 [%u],additional information2[%u],axis[%u]",
													rc_status.wrn_info.msg.c_str(),rc_status.wrn_num,rc_status.wrn_info.add_info1,rc_status.wrn_info.add_info2,rc_status.wrn_info.jnt);

			pub_rcstatus=true;
		}

		if(pub_rcstatus) rc_state_publisher_func();

	}

	rclcpp::Node::SharedPtr& EpsonRobotDriver::get_node(void){
		return node_;
	}
	

	void EpsonRobotDriver::terminate(void){
		RCLCPP_FATAL(logger_, "terminate is called.");

		short ret=0;
		//Disable real-time motion command control mode
		ret=rtmc_client_->Set_RTMCModeDisable();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_RTMCModeDisable");
		}

		//De-energize the motor
		ret=rtmc_client_->MotorOff();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"MotorOff");
		}

		//Disconnect from the controller
		ret=rtmc_client_->RTMC_DisConnect();
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"RTMC_DISConnect");
			if(ret==CLIENT_ERR)  RCLCPP_FATAL(logger_, "The Connection to epson robot controller was forcibly terminated");
		}

		return; // Use "return" here. Don't use "exit()" to proceed process in caller function.
	}

	void EpsonRobotDriver::Set_SendRecvMode()
	{

		bitset<16> send_format(rtmc_send_format);
		bitset<16> recv_format(rtmc_recv_format);
		bitset<32> sendrecv_format(send_format.to_string()+recv_format.to_string());

		sendrecv_mode=sendrecv_format.to_ulong();
	}


	void EpsonRobotDriver::Prepare_ServiceTopic()
	{
		node_ = rclcpp::Node::make_shared(epson_service_topic_name);
		
			
				RBCheck_service_ =
				node_->create_service<epson_robot_msgs::srv::RBCheck>
				(epson_service_topic_name+"/rb_check",
				std::bind(&EpsonRobotDriver::RBCheck_service_bound_func,
						this, std::placeholders::_1, std::placeholders::_2));
			

		
				RBCheckResult_service_ =
				node_->create_service<epson_robot_msgs::srv::RBCheckResult>
				(epson_service_topic_name+"/rb_check_result",
				std::bind(&EpsonRobotDriver::RBCheckResult_service_bound_func,
						this, std::placeholders::_1, std::placeholders::_2));
			

				Get_RBModel_service_ =
				node_->create_service<epson_robot_msgs::srv::GetRBModel>
				(epson_service_topic_name+"/get_rb_model",
				std::bind(&EpsonRobotDriver::Get_RBModel_service_bound_func,
						this, std::placeholders::_1, std::placeholders::_2));
			

				Set_RTMCSendFormat_service_ =
				node_->create_service<epson_robot_msgs::srv::SetRTMCSendFormat>
				(epson_service_topic_name+"/set_rtmc_send_format",
				std::bind(&EpsonRobotDriver::Set_RTMCSendFormat_service_bound_func,
						this, std::placeholders::_1, std::placeholders::_2));
			

				Get_RTMCSendFormat_service_ =
				node_->create_service<epson_robot_msgs::srv::GetRTMCSendFormat>
				(epson_service_topic_name+"/get_rtmc_send_format",
				std::bind(&EpsonRobotDriver::Get_RTMCSendFormat_service_bound_func,
						this, std::placeholders::_1, std::placeholders::_2));
			

				Set_RTMCRecvFormat_service_ =
				node_->create_service<epson_robot_msgs::srv::SetRTMCRecvFormat>
				(epson_service_topic_name+"/set_rtmc_recv_format",
				std::bind(&EpsonRobotDriver::Set_RTMCRecvFormat_service_bound_func,
						this, std::placeholders::_1, std::placeholders::_2));
			

			Get_RTMCRecvFormat_service_ =
			node_->create_service<epson_robot_msgs::srv::GetRTMCRecvFormat>
			(epson_service_topic_name+"/get_rtmc_recv_format",
			 std::bind(&EpsonRobotDriver::Get_RTMCRecvFormat_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));
			
			MotorOn_service_ =
			node_->create_service<epson_robot_msgs::srv::MotorOn>
			(epson_service_topic_name+"/motor_on",
			 std::bind(&EpsonRobotDriver::MotorOn_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));
			
			MotorOff_service_ =
			node_->create_service<epson_robot_msgs::srv::MotorOff>
			(epson_service_topic_name+"/motor_off",
			 std::bind(&EpsonRobotDriver::MotorOff_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_MotorStatus_service_ =
			node_->create_service<epson_robot_msgs::srv::GetMotorStatus>
			(epson_service_topic_name+"/get_motor_status",
			 std::bind(&EpsonRobotDriver::Get_MotorStatus_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));
	   
			PowerHigh_service_ =
			node_->create_service<epson_robot_msgs::srv::PowerHigh>
			(epson_service_topic_name+"/power_high",
			 std::bind(&EpsonRobotDriver::PowerHigh_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			PowerLow_service_ =
			node_->create_service<epson_robot_msgs::srv::PowerLow>
			(epson_service_topic_name+"/power_low",
			 std::bind(&EpsonRobotDriver::PowerLow_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));
 
			Get_PowerMode_service_ =
			node_->create_service<epson_robot_msgs::srv::GetPowerMode>
			(epson_service_topic_name+"/get_power_mode",
			 std::bind(&EpsonRobotDriver::Get_PowerMode_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));


			Set_RTMCModeEnable_service_ =
			node_->create_service<epson_robot_msgs::srv::SetRTMCModeEnable>
			(epson_service_topic_name+"/set_rtmc_mode_enable",
			 std::bind(&EpsonRobotDriver::Set_RTMCModeEnable_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Set_RTMCModeDisable_service_ =
			node_->create_service<epson_robot_msgs::srv::SetRTMCModeDisable>
			(epson_service_topic_name+"/set_rtmc_mode_disable",
			 std::bind(&EpsonRobotDriver::Set_RTMCModeDisable_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_RTMCMode_service_ =
			node_->create_service<epson_robot_msgs::srv::GetRTMCMode>
			(epson_service_topic_name+"/get_rtmc_mode",
			 std::bind(&EpsonRobotDriver::Get_RTMCMode_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_CurrentJA_service_ =
			node_->create_service<epson_robot_msgs::srv::GetCurrentJA>
			(epson_service_topic_name+"/get_current_ja",
			 std::bind(&EpsonRobotDriver::Get_CurrentJA_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

		    Reset_service_ =
			node_->create_service<epson_robot_msgs::srv::Reset>
			(epson_service_topic_name+"/reset",
			 std::bind(&EpsonRobotDriver::Reset_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Set_DigitalOutput_Bit_service_ =
			node_->create_service<epson_robot_msgs::srv::SetDigitalOutputBit>
			(epson_service_topic_name+"/set_digital_output_bit",
			 std::bind(&EpsonRobotDriver::Set_DigitalOutput_Bit_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Set_DigitalOutput_Byte_service_ =
			node_->create_service<epson_robot_msgs::srv::SetDigitalOutputByte>
			(epson_service_topic_name+"/set_digital_output_byte",
			 std::bind(&EpsonRobotDriver::Set_DigitalOutput_Byte_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));
			
			Get_DigitalOutput_Bit_service_ =
			node_->create_service<epson_robot_msgs::srv::GetDigitalOutputBit>
			(epson_service_topic_name+"/get_digital_output_bit",
			 std::bind(&EpsonRobotDriver::Get_DigitalOutput_Bit_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_DigitalOutput_Byte_service_ =
			node_->create_service<epson_robot_msgs::srv::GetDigitalOutputByte>
			(epson_service_topic_name+"/get_digital_output_byte",
			 std::bind(&EpsonRobotDriver::Get_DigitalOutput_Byte_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_DigitalOutput_Word_service_ =
			node_->create_service<epson_robot_msgs::srv::GetDigitalOutputWord>
			(epson_service_topic_name+"/get_digital_output_word",
			 std::bind(&EpsonRobotDriver::Get_DigitalOutput_Word_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_DigitalInput_Bit_service_ =
			node_->create_service<epson_robot_msgs::srv::GetDigitalInputBit>
			(epson_service_topic_name+"/get_digital_input_bit",
			 std::bind(&EpsonRobotDriver::Get_DigitalInput_Bit_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_DigitalInput_Byte_service_ =
			node_->create_service<epson_robot_msgs::srv::GetDigitalInputByte>
			(epson_service_topic_name+"/get_digital_input_byte",
			 std::bind(&EpsonRobotDriver::Get_DigitalInput_Byte_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_DigitalInput_Word_service_ =
			node_->create_service<epson_robot_msgs::srv::GetDigitalInputWord>
			(epson_service_topic_name+"/get_digital_input_word",
			 std::bind(&EpsonRobotDriver::Get_DigitalInput_Word_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Terminate_service_ =
			node_->create_service<epson_robot_msgs::srv::Terminate>
			(epson_service_topic_name+"/terminate",
			 std::bind(&EpsonRobotDriver::Terminate_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));
			
			Set_Weight_service_ =
			node_->create_service<epson_robot_msgs::srv::SetWeight>
			(epson_service_topic_name+"/set_weight",
			 std::bind(&EpsonRobotDriver::Set_Weight_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_Weight_service_ =
			node_->create_service<epson_robot_msgs::srv::GetWeight>
			(epson_service_topic_name+"/get_weight",
			 std::bind(&EpsonRobotDriver::Get_Weight_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Set_Inertia_service_ =
			node_->create_service<epson_robot_msgs::srv::SetInertia>
			(epson_service_topic_name+"/set_inertia",
			 std::bind(&EpsonRobotDriver::Set_Inertia_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));
					   	
			Get_Inertia_service_ =
			node_->create_service<epson_robot_msgs::srv::GetInertia>
			(epson_service_topic_name+"/get_inertia",
			 std::bind(&EpsonRobotDriver::Get_Inertia_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Set_Eccentricity_service_	= 
			node_->create_service<epson_robot_msgs::srv::SetEccentricity>
			(epson_service_topic_name+"/set_eccentricity",
			 std::bind(&EpsonRobotDriver::Set_Eccentricity_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_Eccentricity_service_	= 
			node_->create_service<epson_robot_msgs::srv::GetEccentricity>
			(epson_service_topic_name+"/get_eccentricity",
			 std::bind(&EpsonRobotDriver::Get_Eccentricity_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Set_BufferSize_service_	= 
			node_->create_service<epson_robot_msgs::srv::SetBufferSize>
			(epson_service_topic_name+"/set_buffer_size",
			 std::bind(&EpsonRobotDriver::Set_BufferSize_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));

			Get_BufferSize_service_	= 
			node_->create_service<epson_robot_msgs::srv::GetBufferSize>
			(epson_service_topic_name+"/get_buffer_size",
			 std::bind(&EpsonRobotDriver::Get_BufferSize_service_bound_func,
					   this, std::placeholders::_1, std::placeholders::_2));


			rc_state_publisher_ = node_->create_publisher<epson_robot_msgs::msg::RCStatus>(epson_service_topic_name+"/rc_status", 1);

			digital_output_common_subscriber_=node_->create_subscription<epson_robot_msgs::msg::DigitalOutputCommon>(
			epson_service_topic_name+"/digital_output_common", 1,
			std::bind(&EpsonRobotDriver::digital_output_common_subscriber_func,this, std::placeholders::_1)
			);

		   digital_input_common_publisher_= node_->create_publisher<epson_robot_msgs::msg::DigitalInputCommon>(epson_service_topic_name+"/digital_input_common", 1);

	}

	short EpsonRobotDriver::Prepare_RB_Activate()
	{
		short ret=0;

		//Verify that the robot model specified by the argument matches the one registered in the controller
		ret=rtmc_client_->RBCheck(rb_model);
		if(ret != CLIENT_SUCCESS)
		{
			RCLCPP_ERROR(logger_, "The model registered in the controller differs from the robot,RBCheck,CLIENT_ERR");
			return ret;
		}

		//Specify whether the control target is the robot only, or both the robot and standard I/O output.
		ret=rtmc_client_->Set_RTMCSendFormat(rtmc_send_format);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_RTMCSendFormat");
			return ret;
		}

		//Specify whether the target for current value acquisition is the robot only, or both the robot and standard I/O inputs.
		ret=rtmc_client_->Set_RTMCRecvFormat(rtmc_recv_format);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_RTMCRecvFormat");
			return ret;
		}

		//Specify the buffer size in the controller for storing motion command values.
		ret=rtmc_client_->Set_BufferSize(bufsize);
		if(ret != CLIENT_SUCCESS)
		{
			OutputErrLog(ret,"Set_BufferSize");
			return ret;
		}

		if(!weight.weight_default)
		{
			//Set the weight of the hand attached to the robot's end-effector and the object being gripped
			ret=rtmc_client_->Set_Weight(weight.weight_val);
			if(ret != CLIENT_SUCCESS)
			{
				OutputErrLog(ret,"Set_Weight");
				return ret;
			}
		}

		if( (!inertia.inertia_default) || (!eccentricity.eccentricity_default))
		{
			if(inertia.inertia_default)
			{
				//Retrieve the inertia of the hand attached to the robot's end-effector and the object being gripped.
				ret=rtmc_client_->Get_Inertia(&(inertia.inertia_val));
				if(ret != CLIENT_SUCCESS)
				{
					OutputErrLog(ret,"Get_Inertia");
					return ret;
				}

				//Set the inertia of the hand attached to the robot's end-effector and the object being gripped.
				ret=rtmc_client_->Set_Inertia(inertia.inertia_val,eccentricity.eccentricity_val);
				if(ret != CLIENT_SUCCESS)
				{
					OutputErrLog(ret,"Set_Inertia");
					return ret;
				}

			}else{
				if(eccentricity.eccentricity_default)
				{
					//Set the inertia of the hand attached to the robot's end-effector and the object being gripped.
					ret=rtmc_client_->Set_Inertia(inertia.inertia_val);
					if(ret != CLIENT_SUCCESS)
					{
						OutputErrLog(ret,"Set_Inertia");
						return ret;
					}
				}else{
					//Set the inertia of the hand attached to the robot's end-effector and the object being gripped.
					ret=rtmc_client_->Set_Inertia(inertia.inertia_val,eccentricity.eccentricity_val);
					if(ret != CLIENT_SUCCESS)
					{
						OutputErrLog(ret,"Set_Inertia");
						return ret;
					}
				}
			}


		}
		

		return CLIENT_SUCCESS;
	}

	short EpsonRobotDriver::Wrap_Exec_RTMC(vector<double> cmd_,vector<double>& ppos_)
	{
		short ret=0;
		if(num_joints_ == JOINTS_SCARA) meter2millimeter(&cmd_[2]);

		switch(sendrecv_mode)
		{
			case RTMC_JA :
			ret=rtmc_client_->Exec_RTMC(cmd_,ppos_);
			break;

			case RTMC_JA_DO:
			ret=rtmc_client_->Exec_RTMC(cmd_,ppos_,cmddo);
			break;

			case RTMC_JA_DI:
			ret=rtmc_client_->Exec_RTMC(cmd_,ppos_,&get_di);
			break;

			case RTMC_JA_DODI:
			ret=rtmc_client_->Exec_RTMC(cmd_,ppos_,cmddo,&get_di);
			break;
		}

		if(num_joints_ == JOINTS_SCARA) millimeter2meter(&ppos_[2]);

		return ret;
	}


	short EpsonRobotDriver::Wrap_Get_CurrentJA(vector<double>& pcmd_, vector<double>& ppos_)
	{
		short ret=0;

		ret=rtmc_client_->Get_CurrentJA(ppos_);
		if(ret==CLIENT_SUCCESS)
		{
			if(num_joints_==JOINTS_SCARA) millimeter2meter(&ppos_[2]);
			pcmd_=ppos_;
		}
		if(ret!=CLIENT_RTMC_MODE_ERR)	NeedOutputErrLog_forGetCurrentJA=true;

		return ret;
	}

	void EpsonRobotDriver::OutputErrLog(short err,const string& clientapi)
	{
		switch(err)
		{
			case CLIENT_OPERATION_MODE_ERR :
			RCLCPP_ERROR(logger_, "Command execution failed because epson robot controller's operation mode is not set to Auto mode,%s,CLIENT_OPERATION_MODE_ERR",clientapi.c_str());
			break;

			case CLIENT_RBCHK_ERR :
			RCLCPP_ERROR(logger_, "Command execution failed because the robot model verification is not completed,%s,CLIENT_RBCHK_ERR ",clientapi.c_str());
			break;

			case CLIENT_RTMC_MODE_ERR :
			RCLCPP_ERROR(logger_, "This command cannot be executed in the curreent RT Motion Control mode,%s,CLIENT_RTMC_MODE_ERR",clientapi.c_str());
			break;

			case CLIENT_RCERR_STATUS_ERR :
			RCLCPP_ERROR(logger_, "Command execution failed due to an error in epson robot controller,%s,CLIENT_RCERR_STATUS_ERR",clientapi.c_str());
			break;

			case CLIENT_RCESTOP_STATUS_ERR :
			RCLCPP_ERROR(logger_, "Command execution failed because epson robot controller is in an emergency stop state,%s,CLIENT_RCESTOP_STATUS_ERR",clientapi.c_str());
			break;

			case CLIENT_RCSG_STATUS_ERR :
			RCLCPP_ERROR(logger_, "Command execution failed because safe guard is in the OPEN state,%s,CLIENT_RCSG_STATUS_ERR",clientapi.c_str());
			break;

			case CLIENT_ERR :
			;
			break;

			case CLIENT_COM_ERR :
			RCLCPP_FATAL(logger_, "Command execution failed due to abnormal communication status,%s,CLIENT_COM_ERR",clientapi.c_str());
			break;

			case CLIENT_TIMEOUT_ERR :
			RCLCPP_ERROR(logger_, "Command execution failed due to a timeout ,%s,CLIENT_TIMEOUT_ERR",clientapi.c_str());
			break;

			case CLIENT_ARG_ERR :
			RCLCPP_ERROR(logger_, "Command execution failed because the arguments are not appropriate,%s,CLIENT_ARG_ERR",clientapi.c_str());
			break;

			case CLIENT_UNDEF_ERR :
			RCLCPP_FATAL(logger_, "Command execution failed due to an undefined error,%s,CLIENT_UNDEF_ERR",clientapi.c_str());
			break;

			default :
			RCLCPP_ERROR(logger_, "Error has occurred,%s",clientapi.c_str());
			break;
		}
	}

	void EpsonRobotDriver::meter2millimeter(double* p_scara_j3)
	{
		*p_scara_j3 = (*p_scara_j3)*1000.0;
	}

	void EpsonRobotDriver::millimeter2meter(double* p_scara_j3)
	{
		*p_scara_j3 = (*p_scara_j3)*0.001;
	}

	 void EpsonRobotDriver::RBCheck_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::RBCheck::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::RBCheck::Response> response)
	{
		short ret=rtmc_client_->RBCheck(request->rb_model);
		if(ret==CLIENT_SUCCESS) rb_model=request->rb_model;
		response->res=ret;					 
	}

	void EpsonRobotDriver::RBCheckResult_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::RBCheckResult::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::RBCheckResult::Response> response)
	{
		short ret=rtmc_client_->RBCheckResult();
		response->res=ret;
	}

	void EpsonRobotDriver::Get_RBModel_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRBModel::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetRBModel::Response> response)
	{
		string rb_model="";
		short ret=rtmc_client_->Get_RBModel(&rb_model);
		response->res=ret;
		response->rb_model=rb_model;
	}

	void EpsonRobotDriver::Set_RTMCSendFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCSendFormat::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCSendFormat::Response> response)
	{
		short ret=rtmc_client_->Set_RTMCSendFormat(request->rtmc_send_format);
		if(ret==CLIENT_SUCCESS) rtmc_send_format=request->rtmc_send_format;
		response->res=ret;
		
	}

	void EpsonRobotDriver::Get_RTMCSendFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRTMCSendFormat::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetRTMCSendFormat::Response> response)
	{
		unsigned short temp_send_format=0;
		short ret=rtmc_client_->Get_RTMCSendFormat(&temp_send_format);
		response->res=ret;
		response->rtmc_send_format=temp_send_format;
	}

	void EpsonRobotDriver::Set_RTMCRecvFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCRecvFormat::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCRecvFormat::Response> response)
	{
		short ret=rtmc_client_->Set_RTMCRecvFormat(request->rtmc_recv_format);
		if(ret==CLIENT_SUCCESS) rtmc_recv_format=request->rtmc_recv_format;
		response->res=ret;
	}

	void EpsonRobotDriver::Get_RTMCRecvFormat_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRTMCRecvFormat::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetRTMCRecvFormat::Response> response)
	{
		unsigned short temp_recv_format=0;
		short ret=rtmc_client_->Get_RTMCRecvFormat(&temp_recv_format);
		response->res=ret;
		response->rtmc_recv_format=temp_recv_format;
	}

	void EpsonRobotDriver::MotorOn_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::MotorOn::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::MotorOn::Response> response)
	{
		short ret=rtmc_client_->MotorOn();
		response->res=ret;
	}

	void EpsonRobotDriver::MotorOff_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::MotorOff::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::MotorOff::Response> response)
	{
		short ret=rtmc_client_->MotorOff();
		response->res=ret;
	}

	void EpsonRobotDriver::Get_MotorStatus_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetMotorStatus::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetMotorStatus::Response> response)
	{
		bool temp_motor_status=false;
		short ret=rtmc_client_->Get_MotorStatus(&temp_motor_status);
		response->res=ret;
		response->motor_status=static_cast<uint8_t>(temp_motor_status);
	}

	void EpsonRobotDriver::PowerHigh_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::PowerHigh::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::PowerHigh::Response> response)
	{
		short ret=rtmc_client_->PowerHigh();
		response->res=ret;
	}

	void EpsonRobotDriver::PowerLow_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::PowerLow::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::PowerLow::Response> response)
	{
		short ret=rtmc_client_->PowerLow();
		response->res=ret;
	}

	void EpsonRobotDriver::Get_PowerMode_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetPowerMode::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetPowerMode::Response> response)
	{
		bool power_mode=false;
		short ret=rtmc_client_->Get_PowerMode(&power_mode);
		response->res=ret;
		response->power_mode=static_cast<uint8_t>(power_mode);
	}

	void EpsonRobotDriver::Set_RTMCModeEnable_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeEnable::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeEnable::Response> response)
	{
		short ret=rtmc_client_->Set_RTMCModeEnable();
		response->res=ret;
	}

	void EpsonRobotDriver::Set_RTMCModeDisable_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeDisable::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::SetRTMCModeDisable::Response> response)
	{
		short ret=rtmc_client_->Set_RTMCModeDisable();
		response->res=ret;
	}

	void EpsonRobotDriver::Get_RTMCMode_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetRTMCMode::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetRTMCMode::Response> response)
	{
		bool rtmc_mode=false;
		short ret=rtmc_client_->Get_RTMCMode(&rtmc_mode);
		response->res=ret;
		response->rtmc_mode=static_cast<uint8_t>(rtmc_mode);
	}

	void EpsonRobotDriver::Get_CurrentJA_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetCurrentJA::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetCurrentJA::Response> response)
	{
		vector<double> temp_curja(num_joints_);
		temp_curja.shrink_to_fit();
		vector<double> temp_cmdja(num_joints_);
		temp_cmdja.shrink_to_fit();

		short ret=Wrap_Get_CurrentJA(temp_cmdja,temp_curja);
		response->res=ret;
		response->current_ja=temp_curja;
	}

	void EpsonRobotDriver::Reset_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::Reset::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::Reset::Response> response)
	{
		short ret=rtmc_client_->Reset();
		response->res=ret;
	}

	void EpsonRobotDriver::Set_DigitalOutput_Bit_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputBit::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputBit::Response> response)
	{
		short ret=rtmc_client_->Set_DigitalOutput_Bit(request->bitnum,request->bitdata);
		response->res=ret;
	}

	void EpsonRobotDriver::Set_DigitalOutput_Byte_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputByte::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetDigitalOutputByte::Response> response)
	{
		short ret=rtmc_client_->Set_DigitalOutput_Byte(request->bytenum,request->bytedata);
		response->res=ret;
	}

	void EpsonRobotDriver::Get_DigitalOutput_Bit_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputBit::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputBit::Response> response)
	{
		uint8_t temp_bitdata=0;

		short ret=rtmc_client_->Get_DigitalOutput_Bit(request->bitnum,&temp_bitdata);
		response->res=ret;
		response->bitdata=temp_bitdata;
	}

	void EpsonRobotDriver::Get_DigitalOutput_Byte_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputByte::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputByte::Response> response)
	{
		uint8_t temp_bytedata=0;

		short ret=rtmc_client_->Get_DigitalOutput_Byte(request->bytenum,&temp_bytedata);
		response->res=ret;
		response->bytedata=temp_bytedata;
	}

	void EpsonRobotDriver::Get_DigitalOutput_Word_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputWord::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalOutputWord::Response> response)
	{
		unsigned short temp_worddata=0;

		short ret=rtmc_client_->Get_DigitalOutput_Word(&temp_worddata);
		response->res=ret;
		response->worddata=temp_worddata;
	}

	void EpsonRobotDriver::Get_DigitalInput_Bit_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputBit::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputBit::Response> response)
	{
		uint8_t temp_bitdata=0;

		short ret=rtmc_client_->Get_DigitalInput_Bit(request->bitnum,&temp_bitdata);
		response->res=ret;
		response->bitdata=temp_bitdata;
	}

	void EpsonRobotDriver::Get_DigitalInput_Byte_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputByte::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputByte::Response> response)
	{
		uint8_t temp_bytedata=0;

		short ret=rtmc_client_->Get_DigitalInput_Byte(request->bytenum,&temp_bytedata);
		response->res=ret;
		response->bytedata=temp_bytedata;
	}

	void EpsonRobotDriver::Get_DigitalInput_Word_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputWord::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::GetDigitalInputWord::Response> response)
	{
		unsigned short temp_worddata=0;

		short ret=rtmc_client_->Get_DigitalInput_Word(request->wordnum,&temp_worddata);
		response->res=ret;
		response->worddata=temp_worddata;
	}

	void  EpsonRobotDriver::Terminate_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::Terminate::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::Terminate::Response> /*response*/)
	{
		ExecTerminateProcess=true;
	}

	void EpsonRobotDriver::Set_Weight_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetWeight::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetWeight::Response> response)
	{
		short ret=rtmc_client_->Set_Weight(request->weight);
		if(ret == CLIENT_SUCCESS)
		{
			weight.weight_val=request->weight;
			weight.weight_default=false;
		}

		response->res=ret;
	}

	void EpsonRobotDriver::Get_Weight_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetWeight::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetWeight::Response> response)
	{
		double temp_weight=0.0;
		short ret=rtmc_client_->Get_Weight(&temp_weight);
		response->res=ret;
		response->weight=temp_weight;

	}

	void EpsonRobotDriver::Set_Inertia_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetInertia::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetInertia::Response> response)
	{
		short ret=rtmc_client_->Set_Inertia(request->inertia);
		if(ret == CLIENT_SUCCESS)
		{
			inertia.inertia_val=request->inertia;
			inertia.inertia_default=false;
		}
		response->res=ret;
	}
	

	void EpsonRobotDriver::Get_Inertia_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetInertia::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetInertia::Response> response)
	{
		double temp_inertia=0.0;
		short ret=rtmc_client_->Get_Inertia(&temp_inertia);
		response->res=ret;
		response->inertia=temp_inertia;

	}
	
	void EpsonRobotDriver::Set_Eccentricity_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetEccentricity::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetEccentricity::Response> response)
	{
		short ret=rtmc_client_->Set_Inertia(inertia.inertia_val,request->eccentricity);
		if(ret == CLIENT_SUCCESS)
		{
			eccentricity.eccentricity_val=request->eccentricity;
			eccentricity.eccentricity_default=false;
		}
		response->res=ret;
	}
	

	void EpsonRobotDriver::Get_Eccentricity_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetEccentricity::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetEccentricity::Response> response)
	{
		double temp_inertia=0.0;
		unsigned short temp_eccentricity=0;
		short ret=rtmc_client_->Get_Inertia(&temp_inertia,&temp_eccentricity);
		response->res=ret;
		response->eccentricity=temp_eccentricity;

	}

	void EpsonRobotDriver::Set_BufferSize_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::SetBufferSize::Request> request,
								 std::shared_ptr<epson_robot_msgs::srv::SetBufferSize::Response> response)
	{
		short ret=rtmc_client_->Set_BufferSize(request->buffer_size);
		if(ret == CLIENT_SUCCESS) bufsize=request->buffer_size;
		response->res=ret;
	}
	

	void EpsonRobotDriver::Get_BufferSize_service_bound_func(const std::shared_ptr<epson_robot_msgs::srv::GetBufferSize::Request> /*request*/,
								 std::shared_ptr<epson_robot_msgs::srv::GetBufferSize::Response> response)
	{
		unsigned short temp_buffer_size=0;
		short ret=rtmc_client_->Get_BufferSize(&temp_buffer_size);
		response->res=ret;
		response->buffer_size=temp_buffer_size;

	}


	
} // namespace epson_robot_control
