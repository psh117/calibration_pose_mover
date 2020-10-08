cmake_minimum_required(VERSION 3.4)

project(franka_pose_calibration_mover CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Franka_DIR:PATH=~/libfranka/build)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

include_directories(
${EIGEN3_INCLUDE_DIRS})

target_link_libraries(examples_common PUBLIC Franka::Franka)

  add_executable(main main.cpp)
  target_link_libraries(main Franka::Franka examples_common)

include(GNUInstallDirs)

install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
