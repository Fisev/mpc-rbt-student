cmake_minimum_required(VERSION 3.8)
project(mpc_rbt_student)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(chrono REQUIRED)
#find_package(functional REQUIRED)
#find_package(memory REQUIRED)
#find_package(string REQUIRED)
#
include_directories(PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
	

set(dependecies
	rclcpp
	geometry_msgs
	std_msgs
  mpc_rbt_simulator
  nav_msgs
)

add_library(${PROJECT_NAME} SHARED
	src/KeyboardControl.cpp
	src/Localization.cpp
  src/Planning.cpp
	
  #src/Motion.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependecies})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
 
add_executable(keyboard_control src/keyboard_control_node.cpp)
target_link_libraries(keyboard_control ${PROJECT_NAME})
install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})
 

add_executable(Localization src/localization_node.cpp)
target_link_libraries(Localization ${PROJECT_NAME})
install(TARGETS Localization DESTINATION lib/${PROJECT_NAME})
 

add_executable(planning src/planning_node.cpp)
target_link_libraries(planning ${PROJECT_NAME})
install(TARGETS planning DESTINATION lib/${PROJECT_NAME})
 
target_compile_features(mpc_rbt_student PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files

  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependecies})
ament_export_libraries(${PROJECT_NAME})

ament_package()
