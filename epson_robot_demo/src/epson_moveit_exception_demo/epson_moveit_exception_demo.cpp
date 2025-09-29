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

#include "epson_robot_demo/epson_moveit_exception_demo.hpp"


EpsonMoveitExceptionDemo::EpsonMoveitExceptionDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node), rb_model("")
{
  vel_factor=1.0;
  acc_factor=1.0;
  number_cycles=3;

  node_->declare_parameter("rb_model", rb_model);
  node_->declare_parameter("vel_factor", vel_factor);
  node_->declare_parameter("acc_factor", acc_factor);
  node_->declare_parameter("number_cycles", number_cycles);

  EpsonRBPickPlacePoint_Ptr = nullptr;
  EpsonRBPickPlacePoint_Ptr=new EpsonRBPickPlacePoint();

  ExceptionState=false;
}

EpsonMoveitExceptionDemo::~EpsonMoveitExceptionDemo()
{
  delete EpsonRBPickPlacePoint_Ptr;
  EpsonRBPickPlacePoint_Ptr=nullptr;
}

void EpsonMoveitExceptionDemo::Wait_Until_RBMoveFin()
{
  std::vector<double> Prev_joint_values = moveGroup->getCurrentJointValues();

    while (true) {
      
      usleep(1000);
      std::vector<double> Current_joint_values = moveGroup->getCurrentJointValues();

      long unsigned int stop_joint_count = 0;
      for (long unsigned int i = 0; i < Current_joint_values.size(); i++) 
      {
        if (std::abs(Prev_joint_values[i] - Current_joint_values[i]) < 0.0001) ++stop_joint_count;
      }
      if (stop_joint_count == Current_joint_values.size()) break;
      Prev_joint_values=Current_joint_values;
    }
}

bool EpsonMoveitExceptionDemo::EpsonRBPlanMove(vector<double> pointdata)
{
  //Specify the target pose using the MoveIt API
  moveGroup->setJointValueTarget(pointdata);
  //Set the planning time for computing the path to the target pose
  moveGroup->setPlanningTime(5.0);
  //Plan the path to the target pose
  moveGroup->plan(movePlan);
  //Execute the motion using the MoveIt API
  moveit::core::MoveItErrorCode moveit_err=moveGroup->move();
  //Wait for the robot to reach the target pose
  if(moveit_err == moveit::core::MoveItErrorCode::SUCCESS) Wait_Until_RBMoveFin();

  if(ExceptionState) return false;

  return true;
}

void EpsonMoveitExceptionDemo::Prepare_Subscribe_Topic()
{

	rc_state_subscriber_=node_->create_subscription<epson_robot_msgs::msg::RCStatus>(
	"epson_robot_control/rc_status", 1,
	std::bind(&EpsonMoveitExceptionDemo::rc_state_subscriber_func,this, std::placeholders::_1));
}


void EpsonMoveitExceptionDemo::run()
{
  PickPlacePoint epson_rb_pointdata;
  // Retrieve the launch file arguments of the demo program
  node_->get_parameter("rb_model", rb_model);
  RCLCPP_INFO(Logger, "EpsonMoveitExceptionDemo - Robot Model : %s", rb_model.c_str());

  node_->get_parameter("vel_factor", vel_factor);
  RCLCPP_INFO(Logger, "EpsonMoveitExceptionDemo - Velocity Factor : %f", vel_factor);

  node_->get_parameter("acc_factor", acc_factor);
  RCLCPP_INFO(Logger, "EpsonMoveitExceptionDemo - Accel Factor : %f", acc_factor);

  node_->get_parameter("number_cycles", number_cycles);
  RCLCPP_INFO(Logger, "EpsonMoveitExceptionDemo - The number of cycles : %d", number_cycles);

  if(!EpsonRBPickPlacePoint_Ptr->CanLoadPickPlacePoint(rb_model))
  {
    RCLCPP_ERROR(Logger,"This robot model is not supported");
    return;
  }

  moveGroup=make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PlanningGroup);
  //Set velocity scaling in MoveIt
  moveGroup->setMaxVelocityScalingFactor(vel_factor);
  //Set acceleration scaling in MoveIt
  moveGroup->setMaxAccelerationScalingFactor(acc_factor);
  //Load the robot's target pose
  epson_rb_pointdata=EpsonRBPickPlacePoint_Ptr->LoadPickPlacePoint(rb_model);
    
  moveit::planning_interface::MoveGroupInterface::Plan movePlan;
  bool result=false;
  int cycle=0;
  int timeout_cnt=0;
  
  for (cycle = 0; cycle < number_cycles; cycle++)
  {
    try{
      //Compute the path to the target pose and execute the motion command
      result=EpsonRBPlanMove(epson_rb_pointdata.home);
      if(!result) throw result;

      result=EpsonRBPlanMove(epson_rb_pointdata.p1_point);
      if(!result) throw result;

      result=EpsonRBPlanMove(epson_rb_pointdata.home);
      if(!result) throw result;
      
      result=EpsonRBPlanMove(epson_rb_pointdata.p2_point);
      if(!result) throw result;
      RCLCPP_INFO(Logger, "------------------%d cycles completed ------------------", cycle + 1);
    }
    catch(bool)
    {
    
    //Send the current pose to the JointTrajectoryController to cancel the trajectory
	  send_current_traj_msg();
    //Transition the hardware component to the deactivate state
    controller_manager_activate(false);

    do{
		  timeout_cnt++;

      RCLCPP_INFO(Logger,"Please execute [reset] service after sloving the issue that caused the robot motion to fail"); 
      this_thread::sleep_for(chrono::seconds(1));

		  if(timeout_cnt >=10) break;
	
    }while(ExceptionState);

	  if(!ExceptionState)
	  {
      //Transition the hardware component to the activate state
		  controller_manager_activate(true);
		  cycle += -1;
		  continue;
	  }

	  break;
    }
    
  }

  if(cycle == number_cycles)
  {
    RCLCPP_INFO(Logger, "All cycles completed");
  }else{
    RCLCPP_INFO(Logger, "The cycle was aborted");
  }
 
  RCLCPP_INFO(Logger, "Please input [ctrl + c] to exit");
}

void EpsonMoveitExceptionDemo::rc_state_subscriber_func(const epson_robot_msgs::msg::RCStatus rcstatus)
{
  if(rcstatus.safeguard == RC_SG_OPEN || rcstatus.estop == RC_ESTOP_ON || rcstatus.err_num > RC_NONERR) ExceptionState=true;

  if((rcstatus.safeguard != RC_SG_OPEN) && (rcstatus.estop != RC_ESTOP_ON) && (rcstatus.err_num == RC_NONERR)) ExceptionState=false;

}

void EpsonMoveitExceptionDemo::controller_manager_activate(bool active)
{
		rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
			controller_manager_activate_client =
			node_->create_client
			<controller_manager_msgs::srv::SetHardwareComponentState>
			("/controller_manager/set_hardware_component_state");

		
		while(!controller_manager_activate_client->wait_for_service
			   (std::chrono::seconds(1))){
			if(!rclcpp::ok()){
				RCLCPP_FATAL(Logger,
							 "[controller_manager_activate_client] Interrupted while waiting for the service.");
				exit(ERROR_CONTROLLER_MANAGER_ACTIVATE_CLIENT_TIMEOUT);
			}
			RCLCPP_WARN(Logger,
						"[controller_manager_activate_client] service not available, waiting again...");
		}
	
		std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request>
			controller_manager_activate_request =
			std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
		controller_manager_activate_request->name = "";
		if(true == active){
			controller_manager_activate_request->target_state.id =
				lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

			controller_manager_activate_request->target_state.label =
				hardware_interface::lifecycle_state_names::ACTIVE;
		}else{
			controller_manager_activate_request->target_state.id =
				lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

			controller_manager_activate_request->target_state.label =
				hardware_interface::lifecycle_state_names::INACTIVE;
		}

		
		auto controller_manager_activate_result_ =
			controller_manager_activate_client->async_send_request
			(controller_manager_activate_request);

		if((controller_manager_activate_result_.wait_for(std::chrono::seconds(3)) == std::future_status::ready))
		{

			if(true == controller_manager_activate_result_.get()->ok){
				RCLCPP_INFO(Logger, "controller_manager_activate: success");
			}else{
				RCLCPP_ERROR(Logger, "controller_manager_activate: fail");
			}
		}else{
			RCLCPP_FATAL(Logger, "[controller_manager_activate_client] Failed to call service.");
			exit(ERROR_CONTROLLER_MANAGER_ACTIVATE_SEND_REQUEST);
		}

		return;
}

void EpsonMoveitExceptionDemo::send_current_traj_msg(void)
{
		rclcpp::QoS qos(rclcpp::KeepLast{10});
		rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
			joint_trajectory_publisher =
			node_->create_publisher<trajectory_msgs::msg::JointTrajectory>
			("/epson_joint_trajectory_controller/joint_trajectory", qos);
		
		std::unique_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory_msg =
			std::make_unique<trajectory_msgs::msg::JointTrajectory>();

		joint_trajectory_msg->header.stamp.sec = 0;
		joint_trajectory_msg->header.stamp.nanosec = 0;
		joint_trajectory_msg->header.frame_id = "";
			
		std::vector<std::string> joint_names_for_size = moveGroup->getJointNames();
		unsigned long joints_size = joint_names_for_size.size();
		std::vector<std::string> joint_names(joints_size);
		std::vector<trajectory_msgs::msg::JointTrajectoryPoint>
			current_point(1);

		rclcpp::Node::SharedPtr joint_states_subscriber_node =
			rclcpp::Node::make_shared("joint_states_subscriber");

    sensor_msgs::msg::JointState joint_states_msg_;

		bool message_received =
			rclcpp::wait_for_message(joint_states_msg_, joint_states_subscriber_node,
									 "/joint_states", std::chrono::milliseconds(1000));
		if(true == message_received)
    {
		  std::vector<double> current_position(joints_size);
		  if(0 < joints_size){
			  for(unsigned int i=0; i<joints_size; i++){
				joint_names[i] = joint_states_msg_.name[i];
				current_position[i] = joint_states_msg_.position[i];
			}
			current_point[0].positions = current_position;
			current_point[0].time_from_start.sec = 0;
			current_point[0].time_from_start.nanosec = 0;
		}

		joint_trajectory_msg->joint_names = joint_names;
		joint_trajectory_msg->points = current_point;
		joint_trajectory_publisher->publish(std::move(joint_trajectory_msg));
  }else{
    RCLCPP_ERROR(Logger, "wait_for_message failed");
  }
		return;
}


