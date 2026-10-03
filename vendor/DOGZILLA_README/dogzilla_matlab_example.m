% =========================================================
% ROS 2 - Robot Dog Velocity Control Example (MATLAB)
% =========================================================
% This script demonstrates how to connect to the robot dog
% and send movement commands via ROS 2.
%
% Make sure your PC and the robot dog are on the SAME network
% and share the same ROS_DOMAIN_ID before running this script.
% =========================================================

% Set the ROS 2 Domain ID
% Both this machine and the robot dog must use the same ID
% to discover each other on the network.
setenv('ROS_DOMAIN_ID', '0');

% Create a ROS 2 node for this MATLAB session
% Node name: /test_node
node = ros2node("/test_node");

% Create a publisher that sends commands to the robot dog
% Topic  : /cmd_vel
% Type   : geometry_msgs/Twist
%
% The robot dog's driver_node subscribes to this topic
% and translates incoming messages into physical motion.
pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");

% Create an empty Twist message
% A Twist message contains two 3D vectors:
%   linear  (x, y, z) – translational velocity
%   angular (x, y, z) – rotational velocity
msg = ros2message(pub);

% ---- Motion control reference --------------------------------
%  linear.x  > 0  →  Move FORWARD
%  linear.x  < 0  →  Move BACKWARD
%  linear.y  > 0  →  Strafe LEFT
%  linear.y  < 0  →  Strafe RIGHT
%  angular.z > 0  →  Rotate LEFT  (counter-clockwise)
%  angular.z < 0  →  Rotate RIGHT (clockwise)
%  All values = 0 →  STOP
%
% Note: The hardware driver uses a fixed actuation parameter of 10.
%       Only the SIGN of each value matters (positive / negative / zero).
% --------------------------------------------------------------

% Example: Stop the robot dog (all fields default to 0.0)
msg.linear.x  = 0.0;   % no forward / backward motion
msg.linear.y  = 0.0;   % no lateral motion
msg.angular.z = 0.0;   % no rotation

% Publish the message to /cmd_vel
% The driver_node will receive this and command the robot accordingly.
send(pub, msg);