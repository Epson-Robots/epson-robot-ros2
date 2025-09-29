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

#include "epson_robot_demo/epson_moveit_demo.hpp"

EpsonMoveitDemo::EpsonMoveitDemo(const rclcpp::Node::SharedPtr& node)
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

EpsonMoveitDemo::~EpsonMoveitDemo()
{
  delete EpsonRBPickPlacePoint_Ptr;
  EpsonRBPickPlacePoint_Ptr=nullptr;
}

void EpsonMoveitDemo::Wait_Until_RBMoveFin()
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

bool EpsonMoveitDemo::EpsonRBPlanMove(vector<double> pointdata)
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

void EpsonMoveitDemo::Prepare_Subscribe_Topic()
{

	rc_state_subscriber_=node_->create_subscription<epson_robot_msgs::msg::RCStatus>(
	"epson_robot_control/rc_status", 1,
	std::bind(&EpsonMoveitDemo::rc_state_subscriber_func,this, std::placeholders::_1));
}

void EpsonMoveitDemo::run()
{
  PickPlacePoint epson_rb_pointdata;

  // Retrieve the launch file arguments of the demo program
  node_->get_parameter("rb_model", rb_model);
  RCLCPP_INFO(Logger, "EpsonMoveitDemo - Robot Model : %s", rb_model.c_str());

  node_->get_parameter("vel_factor", vel_factor);
  RCLCPP_INFO(Logger, "EpsonMoveitDemo - Velocity Factor : %f", vel_factor);

  node_->get_parameter("acc_factor", acc_factor);
  RCLCPP_INFO(Logger, "EpsonMoveitDemo - Accel Factor : %f", acc_factor);

  node_->get_parameter("number_cycles", number_cycles);
  RCLCPP_INFO(Logger, "EpsonMoveitDemo - The number of cycles : %d", number_cycles);

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
  int cycle=0;
  
  for (cycle = 0; cycle < number_cycles; cycle++)
  {
    //Compute the path to the target pose and execute the motion command
    bool result=EpsonRBPlanMove(epson_rb_pointdata.home);
    if(!result) break;

    result=EpsonRBPlanMove(epson_rb_pointdata.p1_point);
    if(!result) break;

     result=EpsonRBPlanMove(epson_rb_pointdata.home);
    if(!result) break;
    result=EpsonRBPlanMove(epson_rb_pointdata.p2_point);
    if(!result) break;

    RCLCPP_INFO(Logger, "------------------%d cycles completed ------------------", cycle + 1);
  }

  if(cycle == number_cycles)
  {
    RCLCPP_INFO(Logger, "All cycles completed");
  }else{
    RCLCPP_INFO(Logger, "The cycle was aborted");
  }

  RCLCPP_INFO(Logger, "Please input [ctrl + c] to exit");
}

void EpsonMoveitDemo::rc_state_subscriber_func(const epson_robot_msgs::msg::RCStatus rcstatus)
{
  if(rcstatus.safeguard == RC_SG_OPEN || rcstatus.estop == RC_ESTOP_ON || rcstatus.err_num > RC_NONERR) ExceptionState=true;

  if((rcstatus.safeguard != RC_SG_OPEN) && (rcstatus.estop != RC_ESTOP_ON) && (rcstatus.err_num == RC_NONERR)) ExceptionState=false;

}




