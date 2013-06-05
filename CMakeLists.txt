cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

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
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_library(${PROJECT_NAME} src/planar_seg.cpp src/find_extents.cpp src/pass_through_gen.cpp
src/set_boundaries.cpp src/move_arm.cpp src/cluster.cpp src/robot_head.cpp src/incontact.cpp src/set_marker.cpp)

rosbuild_add_executable(read_pcd src/pcd_from_kinect.cpp)
#rosbuild_add_executable(tumble_kinect src/tumble_kinect.cpp)

rosbuild_add_executable(process_pcd src/process_pcd.cpp)
target_link_libraries(process_pcd ${PROJECT_NAME})

rosbuild_add_executable(process_pcd_naive src/process_pcd_naive.cpp)
target_link_libraries(process_pcd_naive ${PROJECT_NAME})

#rosbuild_add_executable(process_pcd_alt src/process_pcd_alt.cpp)
#target_link_libraries(process_pcd_alt ${PROJECT_NAME})

#rosbuild_add_executable(ppt src/pick_place_try.cpp)

##rosbuild_add_executable(pick_n_place src/pick_n_place.cpp)
##rosbuild_add_executable(pick_n_place src/pick_n_place_alt.cpp)
##target_link_libraries(pick_n_place ${PROJECT_NAME})

# src/pass_through_gen.cpp 
#src/planar_seg.cpp src/find_extents.cpp src/find_state.cpp 
#src/cluster.cpp src/extract_color_clusters.cpp src/incontact.cpp src/find_regions.cpp
#src/get_object_cloud.cpp src/set_boundaries.cpp src/move_arm.cpp)

rosbuild_add_executable(mt src/manipulator_try.cpp)
target_link_libraries(mt ${PROJECT_NAME})

rosbuild_add_executable(cartesian_controller src/ik_trajectory_tutorial.cpp)

##rosbuild_add_executable(tumble src/tumble_server.cpp)
##target_link_libraries(tumble ${PROJECT_NAME})
# src/move_arm_tumble.cpp 
#src/find_waypoints.cpp src/set_marker.cpp)

##rosbuild_add_executable(st src/spread_try.cpp)
##target_link_libraries(st ${PROJECT_NAME})

##rosbuild_add_executable(spread src/spread_server.cpp)
##target_link_libraries(spread ${PROJECT_NAME})
# src/move_arm_spread.cpp 
#src/find_circular_path.cpp src/cluster.cpp src/extract_color_clusters.cpp
#src/incontact.cpp src/set_marker.cpp src/find_extents.cpp)

#rosbuild_add_executable(tumble_pcd src/tumble_process_test.cpp src/pass_through.cpp 
#src/planar_seg.cpp src/cluster.cpp src/extract_color_clusters.cpp src/move_arm_tumble.cpp 
#src/find_waypoints.cpp src/set_marker.cpp)

#rosbuild_add_executable(grasp_duplo src/grasp_duplo_server.cpp)

#rosbuild_add_executable(pick_place_color src/pick_and_place_color.cpp src/move_arm.cpp
#src/set_pickup_goal.cpp src/place_duplo.cpp)

#rosbuild_add_executable(pick_place_color src/pick_and_place_color.cpp src/move_arm.cpp
#src/set_pickup_goal.cpp src/place_duplo.cpp)

#rosbuild_add_executable(pick_place_size src/pick_and_place_size.cpp src/move_arm.cpp
#src/find_dim.cpp)

rosbuild_add_executable(move_arm_manual src/move_arm_manual.cpp)
rosbuild_add_executable(move_arm_manual2 src/move_arm_manual2.cpp)
rosbuild_add_executable(move_arm_manual3 src/move_arm_manual3.cpp)
##rosbuild_add_executable(move_leftarm src/move_leftarm_manual3.cpp)
#rosbuild_add_executable(test_move_arm_spread src/move_arm_spread_test.cpp)

#rosbuild_add_executable(move_head src/move_head.cpp)

rosbuild_add_executable(move_head_manual src/move_head_manual.cpp)

#rosbuild_add_executable(open_gripper src/open_gripper.cpp)

rosbuild_add_executable(open_gripper_manual src/open_gripper_manual.cpp)

#rosbuild_add_executable(coll_test src/collision_test.cpp)

#rosbuild_add_executable(my_test src/my_test.cpp src/read_pcd.cpp src/find_waypoints.cpp)

