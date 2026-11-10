cmake_minimum_required(VERSION 3.8)
project(rover_charging_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestCharge.srv"
  "action/ChargeSession.action"
  DEPENDENCIES action_msgs
)

ament_package()
