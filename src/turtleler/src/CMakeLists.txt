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
)

install(TARGETS
    turtle_tf2_broadcaster
    turtle_controller
    boundary_detector
    turtle_brain
    DESTINATION lib/${PROJECT_NAME})
