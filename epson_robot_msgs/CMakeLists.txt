cmake_minimum_required(VERSION 3.8)
project(epson_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/RCStatus.msg"
   "msg/DigitalOutputCommon.msg"
   "msg/DigitalInputCommon.msg"

   "srv/RBCheck.srv"
   "srv/RBCheckResult.srv"
   "srv/GetRBModel.srv"
   "srv/SetRTMCSendFormat.srv"
   "srv/GetRTMCSendFormat.srv"
   "srv/SetRTMCRecvFormat.srv"
   "srv/GetRTMCRecvFormat.srv"
   "srv/MotorOn.srv"
   "srv/MotorOff.srv"
   "srv/GetMotorStatus.srv"
   "srv/PowerHigh.srv"
   "srv/PowerLow.srv"
   "srv/GetPowerMode.srv"
   "srv/SetRTMCModeEnable.srv"
   "srv/SetRTMCModeDisable.srv"
   "srv/GetRTMCMode.srv"
   "srv/GetCurrentJA.srv"
   "srv/Reset.srv"
   "srv/SetDigitalOutputBit.srv"
   "srv/SetDigitalOutputByte.srv"
   "srv/GetDigitalOutputBit.srv"
   "srv/GetDigitalOutputByte.srv"
   "srv/GetDigitalOutputWord.srv"
   "srv/GetDigitalInputBit.srv"
   "srv/GetDigitalInputByte.srv"
   "srv/GetDigitalInputWord.srv"
   "srv/Terminate.srv"
   "srv/SetWeight.srv"
   "srv/GetWeight.srv"
   "srv/SetInertia.srv"
   "srv/GetInertia.srv"
   "srv/SetEccentricity.srv"
   "srv/GetEccentricity.srv"
   "srv/SetBufferSize.srv"
   "srv/GetBufferSize.srv"
  DEPENDENCIES std_msgs
)

ament_package()
