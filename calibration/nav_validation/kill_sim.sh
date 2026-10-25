#!/bin/bash
# Stop every simulation / navigation process by PID (never matches itself); verifies nothing is left.
pids() { ps -eo pid,args | grep -E "gz sim|gz_tools_vendor|ros_gz_bridge|ekf_node|robot_state_publisher|ros2_control_node|ros2 launch|spawner|nav2_|lifecycle_manager|amcl|map_server|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|collision_monitor|docking_server|robot_executor|rviz2|waypoint_follower|smoother_server|route_server|opennav" | grep -v -E "grep|kill_sim" | awk '{print $1}'; }
for p in $(pids); do kill -INT $p 2>/dev/null; done; sleep 4
for p in $(pids); do kill -9 $p 2>/dev/null; done; sleep 2
left=$(pids); [ -z "$left" ] && echo "sim processes: clean" || echo "LEFTOVER: $left"
