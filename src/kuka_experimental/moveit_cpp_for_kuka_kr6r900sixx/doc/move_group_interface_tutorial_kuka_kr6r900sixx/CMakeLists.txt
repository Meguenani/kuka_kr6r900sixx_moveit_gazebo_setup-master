add_executable(move_group_interface_tutorial_kuka_kr6r900sixx src/move_group_interface_tutorial_kuka_kr6r900sixx.cpp  src/compute_rms.cpp)
target_link_libraries(move_group_interface_tutorial_kuka_kr6r900sixx ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
install(TARGETS move_group_interface_tutorial_kuka_kr6r900sixx DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(move_group_interface_tutorial_kuka_kr6r900sixx2 src/move_group_interface_tutorial_kuka_kr6r900sixx2.cpp  src/compute_rms.cpp)
target_link_libraries(move_group_interface_tutorial_kuka_kr6r900sixx2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
install(TARGETS move_group_interface_tutorial_kuka_kr6r900sixx2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(save_kuka_kr6r900sixx_poses src/save_kuka_kr6r900sixx_poses.cpp)
target_link_libraries(save_kuka_kr6r900sixx_poses ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
install(TARGETS save_kuka_kr6r900sixx_poses DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
