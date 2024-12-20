cmake_minimum_required(VERSION 3.5)
project(ares2_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(ares2_msgs REQUIRED) 

# Include directories
include_directories(include)
include_directories(${ament_cmake_INCLUDE_DIRS})
include_directories(${ares2_msgs_INCLUDE_DIRS}) 

# Build the main node executable
add_executable(${PROJECT_NAME}_node src/main.cpp)

# Link the executable with the necessary libraries
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  ares2_msgs 
)
# Path to the tmotor library and include files
set(TMOTOR_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/tmotorcan_cpp/include)
set(TMOTOR_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/tmotorcan_cpp/lib)

# Include the tmotor header files
include_directories(${TMOTOR_INCLUDE_DIR})

# Find and link the tmotor library
find_library(TMOTOR_LIB tmotor PATHS ${TMOTOR_LIBRARY_DIR})

# Error out if tmotor library is not found
if(NOT TMOTOR_LIB)
  message(FATAL_ERROR "tmotor library not found in ${TMOTOR_LIBRARY_DIR}. Please check the path.")
endif()

# Link tmotor library explicitly
target_link_libraries(${PROJECT_NAME}_node ${TMOTOR_LIB})

# Install executables
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
