cmake_minimum_required(VERSION 3.8)
project(ed_rviz_plugins)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
endif()
add_compile_options(-DQT_NO_KEYWORDS)

find_package(ament_cmake REQUIRED)
find_package(ed_gui_server_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/displays/world_model_display.cpp
  src/visuals/entity_visual.cpp
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  ed_gui_server_interfaces::ed_gui_server_interfaces__rosidl_typesupport_cpp
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)
install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(tue_lint_config REQUIRED)

  ament_clang_format(
    CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-format
    --clang-format-version=21
  )
  ament_clang_tidy(
    CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-tidy
    --clang-tidy-version=21
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/compile_commands.json
  )
  ament_lint_cmake(MAX_LINE_LENGTH 120)
  ament_xmllint(MAX_LINE_LENGTH 120)
endif()

ament_package()
