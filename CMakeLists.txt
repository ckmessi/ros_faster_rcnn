cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


find_package(catkin REQUIRED)

rosbuild_init()
find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED camera_calibration_parsers cv_bridge image_transport message_filters nodelet rosconsole roscpp)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)


#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(ros_faster_rcnn src/ros_faster_rcnn.cpp src/ObjectInfo.cpp)
target_link_libraries(ros_faster_rcnn ${catkin_LIBRARIES}
				    ${GTK_LIBRARIES}
                                    ${GTK2_LIBRARIES}
                                    ${OpenCV_LIBRARIES}
                                    ${Boost_LIBRARIES}
)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)
rosbuild_add_executable( KCF src/runtracker.cpp src/kcftracker.cpp src/fhog.cpp )
target_link_libraries( KCF  ${catkin_LIBRARIES}
				${OpenCV_LIBS})

rosbuild_add_executable(KCFClient src/kcf_client.cpp)
target_link_libraries(KCFClient ${catkin_LIBRARIES}
				    ${GTK_LIBRARIES}
                                    ${GTK2_LIBRARIES}
                                    ${OpenCV_LIBRARIES}
                                    ${Boost_LIBRARIES}
)

#target_link_libraries(example ${PROJECT_NAME})
