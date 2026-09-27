cmake_minimum_required(VERSION 3.14)
project(tram_vehicle_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/VelocitySensor.msg
        msg/DriverControllerCommand.msg
        DEPENDENCIES
            "std_msgs"
        )

ament_package()
