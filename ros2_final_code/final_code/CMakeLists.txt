cmake_minimum_required(VERSION 3.8)
project(dsr_rokey2)

# ===========================
# Dependencies
# ===========================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ===========================
# Generate action interfaces
# ===========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ToolPick.action"
  "action/WipePanel.action"
  "action/WaterPatrol.action"
  "msg/SystemState.msg"
  DEPENDENCIES action_msgs std_msgs builtin_interfaces
)

# ===========================
# Install Python executables
# ===========================
install(PROGRAMS
  dsr_rokey2/tool_grip.py
  dsr_rokey2/wipe_floor.py
  dsr_rokey2/water_floor.py
  dsr_rokey2/move_basic.py
  dsr_rokey2/move_periodic.py
  dsr_rokey2/force_test.py
  dsr_rokey2/mini_jog.py
  dsr_rokey2/code_test.py
  dsr_rokey2/cal_angle.py
  dsr_rokey2/cal_length.py
  dsr_rokey2/single_thread_test.py
  dsr_rokey2/main_controller.py
  dsr_rokey2/check_map.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===========================
# Install launch files
# ===========================
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
