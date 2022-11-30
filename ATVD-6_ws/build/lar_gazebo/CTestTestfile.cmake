# CMake generated Testfile for 
# Source directory: /home/and/Github/ENGC68/ATVD-6_ws/src/lar_gazebo
# Build directory: /home/and/Github/ENGC68/ATVD-6_ws/build/lar_gazebo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_lar_gazebo_roslaunch-check_launch "/home/and/Github/ENGC68/ATVD-6_ws/build/lar_gazebo/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/and/Github/ENGC68/ATVD-6_ws/build/lar_gazebo/test_results/lar_gazebo/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/and/Github/ENGC68/ATVD-6_ws/build/lar_gazebo/test_results/lar_gazebo" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/and/Github/ENGC68/ATVD-6_ws/build/lar_gazebo/test_results/lar_gazebo/roslaunch-check_launch.xml\" \"/home/and/Github/ENGC68/ATVD-6_ws/src/lar_gazebo/launch\" ")
set_tests_properties(_ctest_lar_gazebo_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/and/Github/ENGC68/ATVD-6_ws/src/lar_gazebo/CMakeLists.txt;45;roslaunch_add_file_check;/home/and/Github/ENGC68/ATVD-6_ws/src/lar_gazebo/CMakeLists.txt;0;")
subdirs("gtest")
