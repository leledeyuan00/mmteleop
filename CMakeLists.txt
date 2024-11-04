cmake_minimum_required(VERSION 3.5)
project(mmteleop)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  visualization_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  Eigen3
  tf2_eigen
  control_toolbox
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS})

add_executable(mmteleop src/mmteleop.cpp)
ament_target_dependencies(mmteleop ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(mmteleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(bodytf_pub src/bodytf_pub.cpp)
ament_target_dependencies(bodytf_pub ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(bodytf_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(hri_safety src/hri_safety.cpp)
ament_target_dependencies(hri_safety ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(hri_safety PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(hri_cover_plate src/hri_cover_plate.cpp)
ament_target_dependencies(hri_cover_plate ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(hri_cover_plate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(hri_system src/hri_system.cpp)
ament_target_dependencies(hri_system ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(hri_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(TARGETS
  mmteleop
  bodytf_pub
  hri_safety
  hri_cover_plate
  hri_system
  DESTINATION lib/${PROJECT_NAME}
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
