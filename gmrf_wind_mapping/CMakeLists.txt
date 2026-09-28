cmake_minimum_required(VERSION 3.10)
project(gmrf_wind_mapping)


set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(angles REQUIRED) 
find_package(Eigen3 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(visualization_msgs REQUIRED) 
find_package(olfaction_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(gmrf_msgs REQUIRED) 
find_package(OpenCV REQUIRED)


FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")

include_directories(
  include
  )
  
add_executable(gmrf_wind_mapping_node src/gmrf_node.cpp src/gmrf_map.cpp)
target_include_directories(gmrf_wind_mapping_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gmrf_wind_mapping_node ${OpenCV_LIBS})
  
ament_target_dependencies(gmrf_wind_mapping_node
    rclcpp
    angles
    Eigen3
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    olfaction_msgs
    geometry_msgs
    gmrf_msgs
)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_wind_map test/test_wind_map.cpp src/gmrf_map.cpp)
  target_include_directories(test_wind_map PRIVATE src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_wind_map ${OpenCV_LIBS})
  ament_target_dependencies(test_wind_map rclcpp Eigen3 nav_msgs visualization_msgs geometry_msgs tf2_geometry_msgs)
  add_test(NAME wind_map_singular COMMAND $<TARGET_FILE:test_wind_map> singular)
  add_test(NAME wind_map_marker COMMAND $<TARGET_FILE:test_wind_map> marker)
  add_test(NAME wind_intake COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_wind_intake.py
    $<TARGET_FILE:gmrf_wind_mapping_node> $<TARGET_FILE:test_wind_map>)
  add_test(NAME wind_intake_delayed_tf COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_wind_intake.py
    $<TARGET_FILE:gmrf_wind_mapping_node> $<TARGET_FILE:test_wind_map> --delayed-tf)
  set_tests_properties(wind_intake wind_intake_delayed_tf PROPERTIES RUN_SERIAL TRUE TIMEOUT 45)
endif()

install(
  TARGETS gmrf_wind_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()