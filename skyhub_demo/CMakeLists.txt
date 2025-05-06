cmake_minimum_required(VERSION 3.8)
project(skyhub_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ugcs_skyhub REQUIRED)

add_executable(observer
  src/observer.cpp
)

target_link_libraries(observer PRIVATE rclcpp::rclcpp ugcs_skyhub::ugcs_skyhub)

target_include_directories(observer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(gnss-driver
  src/gnss_driver.cpp
  src/nmea.cpp
)

target_link_libraries(gnss-driver PRIVATE rclcpp::rclcpp ugcs_skyhub::ugcs_skyhub)

target_include_directories(gnss-driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  
)

add_executable(demo-log
  src/demo_log.cpp
)

target_link_libraries(demo-log PRIVATE rclcpp::rclcpp ugcs_skyhub::ugcs_skyhub)
target_include_directories(demo-log PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(mouse-log
  src/mouse_log.cpp
)
target_link_libraries(mouse-log PRIVATE rclcpp::rclcpp ugcs_skyhub::ugcs_skyhub)
target_include_directories(mouse-log PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(observer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(gnss-driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(demo-log PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(mouse-log PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#Install binaries into target lib subdirectroy. Lib is a standard place for ROS2 to keep binaries.
install(TARGETS
  observer
  gnss-driver
  demo-log
  mouse-log
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src include
         DESTINATION source/${PROJECT_NAME}
)

install(FILES CMakeLists.txt package.xml
          DESTINATION source/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
