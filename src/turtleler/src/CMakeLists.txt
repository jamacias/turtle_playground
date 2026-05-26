add_executable(turtle_tf2_broadcaster turtle_tf2_broadcaster.cpp)
target_link_libraries(turtle_tf2_broadcaster PRIVATE
  ${geometry_msgs_TARGET}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${turtlesim_msgs_TARGETS}
)

add_executable(turtle_controller turtle_controller.cpp)
target_link_libraries(turtle_controller PRIVATE
  ${geometry_msgs_TARGET}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  ${turtlesim_msgs_TARGETS}
  ${turtleler_msgs_TARGETS}
  control_toolbox::control_toolbox
)

add_executable(boundary_detector boundary_detector.cpp)
target_link_libraries(boundary_detector PRIVATE
  rclcpp::rclcpp
)

add_executable(turtle_brain turtle_brain.cpp)
target_link_libraries(turtle_brain PRIVATE
  ${geometry_msgs_TARGET}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${turtlesim_msgs_TARGETS}
  ${turtleler_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
  ${btcpp_ros2_interfaces_TARGETS}
)

### BT Executor
add_executable(bt_executor bt_executor.cpp)
target_link_libraries(bt_executor PRIVATE
  rclcpp::rclcpp
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
  ${btcpp_ros2_interfaces_TARGETS}
)

### BT Plugins
add_library(go_to_pose_plugin SHARED actions/go_to_pose_action.cpp actions/utils.cpp)
target_compile_definitions(go_to_pose_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go_to_pose_plugin 
  ${geometry_msgs_TARGET}
  rclcpp::rclcpp
  ${turtlesim_msgs_TARGETS}
  ${turtleler_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
  ${btcpp_ros2_interfaces_TARGETS}
  yaml-cpp::yaml-cpp
)

add_library(get_pose_plugin SHARED actions/get_pose_action.cpp)
target_compile_definitions(get_pose_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(get_pose_plugin 
  ${geometry_msgs_TARGET}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${turtlesim_msgs_TARGETS}
  ${turtleler_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
  ${btcpp_ros2_interfaces_TARGETS}
)

add_library(generate_dance_plugin SHARED actions/generate_dance_action.cpp)
target_compile_definitions(generate_dance_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(generate_dance_plugin 
  ${geometry_msgs_TARGET}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  ${turtlesim_msgs_TARGETS}
  ${turtleler_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  ${nav_msgs_TARGETS}
)

install(TARGETS
    turtle_tf2_broadcaster
    turtle_controller
    boundary_detector
    turtle_brain
    bt_executor
    go_to_pose_plugin
    get_pose_plugin
    generate_dance_plugin
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  go_to_pose_plugin
  get_pose_plugin
  generate_dance_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )