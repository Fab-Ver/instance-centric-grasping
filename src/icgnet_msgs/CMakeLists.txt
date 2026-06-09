cmake_minimum_required(VERSION 3.8)
project(icgnet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Grasp.msg"
  "msg/GraspArray.msg"
  "msg/SceneObject.msg"
  "msg/SceneManifest.msg"
  "action/ExecuteGrasp.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
