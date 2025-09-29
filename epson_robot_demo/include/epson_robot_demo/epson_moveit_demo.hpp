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

#include <string>

#include "epson_robot_pickplace_point.hpp"
#include "epson_robot_msgs/msg/rc_status.hpp"
#include "rtmc/rtmc_definition.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/client.hpp>

using namespace std;

class EpsonMoveitDemo
{
private:
rclcpp::Node::SharedPtr node_;
string rb_model;

public:

explicit EpsonMoveitDemo(const rclcpp::Node::SharedPtr& node);
~EpsonMoveitDemo();
EpsonMoveitDemo(const EpsonMoveitDemo&) =delete;
EpsonMoveitDemo& operator=(const EpsonMoveitDemo&) =delete;

void run();
void Prepare_Subscribe_Topic();

private:

double vel_factor;
double acc_factor;
int number_cycles;
bool ExceptionState;

const string PlanningGroup = "arm";
const rclcpp::Logger Logger = rclcpp::get_logger("epson_moveit_demo");
shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup;
  moveit::planning_interface::MoveGroupInterface::Plan movePlan;

EpsonRBPickPlacePoint* EpsonRBPickPlacePoint_Ptr;


bool EpsonRBPlanMove(vector<double> pointdata);
void Wait_Until_RBMoveFin();
rclcpp::Subscription<epson_robot_msgs::msg::RCStatus>::SharedPtr rc_state_subscriber_;
void rc_state_subscriber_func(const epson_robot_msgs::msg::RCStatus rcstatus);


};