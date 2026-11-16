cmake_minimum_required(VERSION 3.16)
project(umi_slam_runner LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
add_executable(umi_slam_runner runner.cc)
target_include_directories(umi_slam_runner PRIVATE ${ORB_ROOT} ${ORB_ROOT}/include
  ${ORB_ROOT}/include/CameraModels ${ORB_ROOT}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(umi_slam_runner PRIVATE ${ORB_ROOT}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
