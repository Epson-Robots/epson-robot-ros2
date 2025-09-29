
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
#include "joint_trajectory_controller/joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace epson_ros2_controller
{

    class EpsonJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
    {
    public :
        EpsonJointTrajectoryController();
        ~EpsonJointTrajectoryController();
        
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        
    private : 
        void Epson_update_pids();
        void reset_scaled_time(rclcpp::Time time);
        void tick_scaled_time();
        rclcpp::Time current_scaled_time();
        rclcpp::Time scaled_time_;

    };

}