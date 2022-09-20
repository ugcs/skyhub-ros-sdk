cmake_minimum_required(VERSION 3.8)
project(skyhub_aeris_strato)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ugcs_skyhub REQUIRED)


add_executable(aeris-strato-driver
  src/strato_driver.cpp
)

target_link_libraries(aeris-strato-driver PRIVATE rclcpp::rclcpp ugcs_skyhub::ugcs_skyhub)

target_include_directories(aeris-strato-driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(aeris-strato-driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#Install binaries into target lib subdirectroy. Lib is a standard place for ROS2 to keep binaries.
install(TARGETS
  aeris-strato-driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
