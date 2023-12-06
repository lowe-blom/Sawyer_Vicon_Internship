// #ifndef INCLUDE_H
// #define INCLUDE_H

// Include file that includes main libraries used in the multiple files
#include <frames.hpp>

#include <string>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <thread>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

#include <intera_core_msgs/IOComponentCommand.h> 
#include <intera_core_msgs/IODeviceStatus.h>
#include <intera_core_msgs/JointCommand.h>
#include <intera_core_msgs/JointLimits.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <kdl_parser/kdl_parser.hpp>
#include <chaindynparam.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolvervel_pinv.hpp>
#include <chainiksolverpos_nr.hpp>

#include <nlohmann/json.hpp>

#define DIST_THRESHOLD 0.1     // distance threshold for object detection
#define ANGLE_THRESHOLD 0.1     // angle threshold for object detection (arbitrary??)

// #endif