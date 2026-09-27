cmake_minimum_required(VERSION 3.8)
# Резервная одометрия трамвая: нода, плагины, офлайн-прогон, отрисовка карты, документация.
project(tram_reserve_odometry)

# Release по умолчанию.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tram_vehicle_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Python-инструменты: rclpy + сообщения (отрисовка карты в rviz2).
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# Офлайн-прогон по bag (reserve_odometry_replay); без rosbag2_cpp не собирается.
find_package(rosbag2_cpp QUIET)

# ---------------------------------------------------------------------------
# core: типы, интерфейсы, реестр плагинов (header-only)
# ---------------------------------------------------------------------------
add_library(tram_reserve_odometry_core INTERFACE)
target_include_directories(tram_reserve_odometry_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tram_reserve_odometry_core INTERFACE cxx_std_17)
target_link_libraries(tram_reserve_odometry_core INTERFACE pluginlib::pluginlib)

# ---------------------------------------------------------------------------
# map: 1D-карта маршрута (кольцо + ветки + остановки)
# ---------------------------------------------------------------------------
add_library(tram_reserve_odometry_map STATIC
  src/map/route_map.cpp)
set_target_properties(tram_reserve_odometry_map PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(tram_reserve_odometry_map PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(tram_reserve_odometry_map tram_reserve_odometry_core)
target_compile_features(tram_reserve_odometry_map PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# plugins: алгоритмы (pluginlib)
# ---------------------------------------------------------------------------
add_library(tram_reserve_odometry_plugins SHARED
  src/plugins/passthrough_preprocessor.cpp
  src/plugins/table_traction_model.cpp
  src/plugins/always_good_slip_detector.cpp
  src/plugins/kalman_wheel_estimator.cpp
  src/plugins/stop_reset_route_integrator.cpp
  src/plugins/zero_position_initializer.cpp)
target_include_directories(tram_reserve_odometry_plugins PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(tram_reserve_odometry_plugins
  tram_reserve_odometry_core
  tram_reserve_odometry_map)
ament_target_dependencies(tram_reserve_odometry_plugins
  pluginlib
  rclcpp)
target_compile_features(tram_reserve_odometry_plugins PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(tram_reserve_odometry plugins.xml)

# ---------------------------------------------------------------------------
# node: ROS-нода (плагины грузит по имени); библиотека — общая с офлайн-прогоном.
# ---------------------------------------------------------------------------
add_library(tram_reserve_odometry_node_lib STATIC
  src/node/reserve_odometry_node.cpp)
set_target_properties(tram_reserve_odometry_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(tram_reserve_odometry_node_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(tram_reserve_odometry_node_lib
  tram_reserve_odometry_core
  tram_reserve_odometry_map)
ament_target_dependencies(tram_reserve_odometry_node_lib
  rclcpp
  pluginlib
  std_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  sensor_msgs
  tram_vehicle_msgs
  ament_index_cpp)
target_compile_features(tram_reserve_odometry_node_lib PUBLIC cxx_std_17)

add_executable(tram_reserve_odometry_node
  src/main.cpp)
target_link_libraries(tram_reserve_odometry_node tram_reserve_odometry_node_lib)
target_compile_features(tram_reserve_odometry_node PRIVATE cxx_std_17)

# Офлайн-прогон: та же нода, сообщения из bag, таймер по времени записи bag.
if(rosbag2_cpp_FOUND)
  add_executable(reserve_odometry_replay
    src/tools/replay_main.cpp)
  target_link_libraries(reserve_odometry_replay tram_reserve_odometry_node_lib)
  ament_target_dependencies(reserve_odometry_replay rosbag2_cpp)
  target_compile_features(reserve_odometry_replay PRIVATE cxx_std_17)
  install(TARGETS reserve_odometry_replay RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "rosbag2_cpp not found: reserve_odometry_replay is not built")
endif()

# ---------------------------------------------------------------------------
# Python-инструменты (отрисовка карты в rviz2)
# ---------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PY_SITE "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

install(DIRECTORY python/${PROJECT_NAME}
  DESTINATION ${PY_SITE}
  FILES_MATCHING PATTERN "*.py")

install(PROGRAMS
  scripts/live_map_viewer
  DESTINATION lib/${PROJECT_NAME})

# Хук pythonpath для `import tram_reserve_odometry`.
set(_HOOK_DIR ${CMAKE_CURRENT_BINARY_DIR}/hook)
file(MAKE_DIRECTORY ${_HOOK_DIR})
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/share/hook/pythonpath.sh.in
  ${_HOOK_DIR}/pythonpath.sh @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/share/hook/pythonpath.dsv.in
  ${_HOOK_DIR}/pythonpath.dsv @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/share/hook/pythonpath.ps1.in
  ${_HOOK_DIR}/pythonpath.ps1 @ONLY)
install(FILES
  ${_HOOK_DIR}/pythonpath.sh
  ${_HOOK_DIR}/pythonpath.dsv
  ${_HOOK_DIR}/pythonpath.ps1
  DESTINATION share/${PROJECT_NAME}/hook)

# ---------------------------------------------------------------------------
# Установка
# ---------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include)
install(TARGETS tram_reserve_odometry_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  tram_reserve_odometry_core
  tram_reserve_odometry_map
  tram_reserve_odometry_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY configs launch
  DESTINATION share/${PROJECT_NAME})
# Карта маршрута и характеристика привода.
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME})

ament_package()
