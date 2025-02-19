cmake_minimum_required(VERSION 2.8.3)

project(my_first_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    gazebo_ros 
    gazebo_ros_control
    controller_manager
    diff_drive_controller
    joint_state_controller
)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS
  scripts/robot_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

