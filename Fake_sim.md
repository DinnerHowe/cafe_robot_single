# amclf仿真
roslaunch rbx1_bringup fake_turtlebot.launch
roslaunch rbx1_nav fake_amcl.launch

roslaunch rbx1_nav fake_amcl_withoutmap.launch
rosrun planner ReadMap.py

roslaunch machine robot_controller.launch
roslaunch marker ui_marker.launch
roslaunch machine 3D_RVIZ.launch
rosrun nav_staff fake_keyboard_control.py
rosrun nav_staff tele_handle_for_rviz.py
rosrun planner interactive_marker
