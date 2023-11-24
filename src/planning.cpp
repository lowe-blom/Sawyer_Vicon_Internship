#include <planning.h>

namespace my_planning
{
    void myPlanningClass::pickAndPlace(const ros::Publisher& pub, geometry_msgs::Pose targetPose)
    {
        double wait = 0.5;
        
        ros::Duration(2).sleep();
        goToPoseGoal(getStartPose());
        ros::Duration(wait).sleep();

        updatePose(targetPose, 0.01, 0.005, 0.11);
        ros::Duration(wait).sleep();

        updatePose(targetPose, 0.01, 0.005, 0.01);
        operateGripper(pub, gripperState);
        ros::Duration(wait).sleep();

        goToPoseGoal(getStartPose());
        ros::Duration(wait).sleep();

        updatePose(targetPose, 0.2, 0.2, 0.11);
        ros::Duration(wait).sleep();

        updatePose(targetPose, 0.2, 0.2, 0.01);
        operateGripper(pub, gripperState);
        ros::Duration(wait).sleep();

        updatePose(targetPose, 0.2, 0.2, 0.11);
        ros::Duration(wait).sleep();

        goToPoseGoal(getStartPose());
        ros::Duration(wait).sleep();
    }

    void myPlanningClass::updatePose(geometry_msgs::Pose targetPose, double x, double y, double z)
    {
        targetPose.position.x += x;
        targetPose.position.y += y;
        targetPose.position.z += z;
        bool rotSuccess = rotatePose(targetPose, 'y', M_PI);

        if (!rotSuccess) throw std::runtime_error("Invalid axis rotation");
        goToPoseGoal(targetPose);
    }

    void myPlanningClass::goToPoseGoal(geometry_msgs::Pose targetPose)
    {
        printPose(targetPose);
        move_group.setPoseTarget(targetPose);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!success) throw std::runtime_error("No plan found");

        move_group.move(); //blocking call (stay in this function until the robot reaches the goal)
    }

    void myPlanningClass::computeTrajectory(geometry_msgs::Pose targetPose)
    {
        printPose(targetPose);
        move_group.setPoseTarget(targetPose);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!success) throw std::runtime_error("No plan found");

        moveit_msgs::RobotTrajectory trajectory_msg = my_plan.trajectory_;

        // Access joint trajectory points
        const std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_points = trajectory_msg.joint_trajectory.points;

        // Iterate through the trajectory points to get position, velocity, and acceleration
        for (size_t i = 0; i < trajectory_points.size(); ++i) 
        {
            std::vector<double> joint_positions = trajectory_points[i].positions;
            std::vector<double> joint_velocities = trajectory_points[i].velocities;
            std::vector<double> joint_accelerations = trajectory_points[i].accelerations;

            // Do something with the position, velocity, and acceleration data
            // For example, print them
            ROS_INFO("Point %zu - Positions: %s, Velocities: %s, Accelerations: %s",
                    i,
                    joint_positions[1],
                    joint_velocities[1],
                    joint_accelerations[1]);
        }
    }

    void myPlanningClass::goToJointState()
    {
        robot_state::RobotState current_state = *move_group.getCurrentState();
        //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        std::vector<double> joint_positions;
        joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
        current_state.copyJointGroupPositions(joint_model_group, joint_positions);
        //joint_positions = move_group.getCurrentJointValues();
        
        joint_positions[0] = -0.1;

        move_group.setJointValueTarget(joint_positions);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(!success) throw std::runtime_error("No plan found");

        move_group.move();
    }

    void myPlanningClass::cartesionPath()
    {
        geometry_msgs::Pose targetPose1 = startPose;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(targetPose1);

        geometry_msgs::Pose targetPose2 = targetPose1;

        targetPose2.position.z -= 0.2;
        waypoints.push_back(targetPose2);

        targetPose2.position.z -= 0.2;
        waypoints.push_back(targetPose2);

        targetPose2.position.z += 0.2;
        targetPose2.position.y += 0.2;
        targetPose2.position.x += 0.2;
        waypoints.push_back(targetPose2);

        move_group.setMaxVelocityScalingFactor(0.1);

        // We want the cartesion path to be interpolated at a resolution of 1 cm
        // which is why we will specify 0.01 as the max step in cartesian
        // translation.  We will specify the jump threshold as 0.0, effectively
        // disabling it. Warning - disabling the jump threshold while operating
        // real hardware can cause large unpredictable motions of redundant
        // joints and could be a safety issue

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        move_group.move();
        ROS_INFO_STREAM("Percentage of path followed: " << fraction * 100.0);
    }

    void myPlanningClass::resetValues()
    {
        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(0.75);
    }

    void myPlanningClass::initGripper(const ros::Publisher& pub)
    {
        ROS_INFO("Activating ClickSmart...");
        intera_core_msgs::IOComponentCommand msg;

        msg.time = ros::Time::now();
        msg.op = "activate";
        msg.args = "{\"devices\": [\"stp_022005TP99124\"]}";

        pub.publish(msg);
        ros::Duration(1).sleep();
    }

    void myPlanningClass::makeBox(std::string blk_name, double *pose)
    {
        moveit_msgs::CollisionObject box;
        box.header.frame_id = move_group.getPlanningFrame();
        box.id = blk_name;

        // Sizes are defined by size and position of table in world
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.7;
        primitive.dimensions[1] = 1.4;
        primitive.dimensions[2] = 0.725;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 0.0;
        box_pose.position.x = pose[0];
        box_pose.position.y = pose[1];
        box_pose.position.z = pose[2];

        box.primitives.push_back(primitive);
        box.primitive_poses.push_back(box_pose);
        box.operation = box.ADD;

        std::vector<moveit_msgs::CollisionObject> collisionObjects;
        collisionObjects.push_back(box);
        ros::Duration(2).sleep();
        virtual_world.addCollisionObjects(collisionObjects);
        ROS_INFO_STREAM("Added box " << blk_name << " into the world");
    }

    void myPlanningClass::addObjects()
    {
        double box_pose1[3] = {0.55, 0.0, -0.56,};
        if (virtual_world.getKnownObjectNames().size() == 0)
        {
            std::cout << "No objects in the world. Now adding table." << std::endl;
            makeBox("table1", box_pose1);
        } else {
            std::vector<std::string> objectNames = virtual_world.getKnownObjectNames();
            std::cout << "Objects in world: ";
            for (const std::string& name : objectNames) {
                std::cout << name << ", ";
            }
            std::cout << std::endl;
            std::cout << "Table already in the world. Not adding a new object" << std::endl;
        }
    }

    void myPlanningClass::removeObjects()
    {
        std::vector<std::string> object_ids;
        object_ids.push_back("table1");
        virtual_world.removeCollisionObjects(object_ids);
        ROS_INFO_STREAM("Removed objects from the world");
    }

    void myPlanningClass::operateGripper(const ros::Publisher& pub, bool gripperState)
    {
        intera_core_msgs::IOComponentCommand msg = makeMessage(gripperState);
        ros::Duration(1).sleep();
        std::string topic = pub.getTopic();

        pub.publish(msg);
        ros::Duration(1).sleep();
    }

    intera_core_msgs::IOComponentCommand myPlanningClass::makeMessage(bool gripperState)
    {
        nlohmann::json json_obj;
        ROS_INFO("Gripper Value: %s", gripperState ? "true" : "false");
        if (!gripperState)
        {
            json_obj = {
                {"signals", {
                    {"grip_HkZFMKl3Wu", {
                        {"data", {true}},
                        {"format", {
                            {"type", "bool"}
                        }}
                    }}
                }}
            };
        } else {
            json_obj = {
                {"signals", {
                    {"grip_HkZFMKl3Wu", {
                        {"data", {false}},
                        {"format", {
                            {"type", "bool"}
                        }}
                    }}
                }}
            };
        }
        
        intera_core_msgs::IOComponentCommand msg;
        msg.time = ros::Time::now();
        msg.op = "set"; 
        msg.args = json_obj.dump();

        return msg;
    }

    void myPlanningClass::printPose(geometry_msgs::Pose objectPose)
    {
        ROS_INFO_STREAM("Pose: " << objectPose);
    }

    bool myPlanningClass::rotatePose(geometry_msgs::Pose& pose, char axis, double angle)
    {
        tf2::Matrix3x3 rotationMatrix(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

        tf2::Quaternion additionalRotation;
        if (axis == 'x') {
            additionalRotation.setRPY(angle, 0.0, 0.0);
        } else if (axis == 'y') {
            additionalRotation.setRPY(0.0, angle, 0.0);
        } else if (axis == 'z') {
            additionalRotation.setRPY(0.0, 0.0, angle);
        } else {
            std::cout << "Invalid axis: Choose one of the following 'x', 'y', 'z'" << std::endl;
            return false;
        }

        rotationMatrix *= tf2::Matrix3x3(additionalRotation);

        tf2::Quaternion q;
        rotationMatrix.getRotation(q);

        q.normalize();
        pose.orientation = tf2::toMsg(q);

        return true;
    }

    geometry_msgs::Pose myPlanningClass::poseCam2Vicon(geometry_msgs::Pose objectPoseEstimate)
    {
        geometry_msgs::Pose objectPoseVicon = getClampPose();
        geometry_msgs::Pose mantaPose = getCameraPose();
        
        //Coordinate transformation from camera frame to Vicon frame.
        tf2::Transform tf_camera;
        tf_camera.setOrigin(tf2::Vector3(mantaPose.position.x, mantaPose.position.y, mantaPose.position.z));
        tf_camera.setRotation(tf2::Quaternion(mantaPose.orientation.x, mantaPose.orientation.y, mantaPose.orientation.z, mantaPose.orientation.w));

        tf2::Transform tf_object;
        tf_object.setOrigin(tf2::Vector3(objectPoseEstimate.position.x, objectPoseEstimate.position.y, objectPoseEstimate.position.z));
        tf_object.setRotation(tf2::Quaternion(objectPoseEstimate.orientation.x, objectPoseEstimate.orientation.y, objectPoseEstimate.orientation.z, objectPoseEstimate.orientation.w));

        tf2::Transform tf_vicon = tf_camera * tf_object;

        geometry_msgs::Pose worldPose;
        worldPose.position.x = tf_vicon.getOrigin().getX();
        worldPose.position.y = tf_vicon.getOrigin().getY();
        worldPose.position.z = tf_vicon.getOrigin().getZ();
        worldPose.orientation.x = tf_vicon.getRotation().getX();
        worldPose.orientation.y = tf_vicon.getRotation().getY();
        worldPose.orientation.z = tf_vicon.getRotation().getZ();
        worldPose.orientation.w = tf_vicon.getRotation().getW();

        //Compare the pose of the object in the Vicon frame with the transformed pose of the object in the camera frame.
        bool comparison = comparePoses(objectPoseVicon, worldPose);

        std::cout << "Comparison result: " << comparison << std::endl;

        if (comparison)
        {
            std::cout << "Pose of object in Vicon frame matches pose of object in camera frame" << std::endl;
        } else {
            std::cout << "Pose of object in Vicon frame does not match pose of object in camera frame" << std::endl;
        }
        return worldPose;
    }

    bool myPlanningClass::comparePoses(geometry_msgs::Pose objectPose, geometry_msgs::Pose objectPoseEstimate)
    {
        double dist = sqrt( pow(objectPose.position.x - objectPoseEstimate.position.x, 2) + 
                            pow(objectPose.position.y - objectPoseEstimate.position.y, 2) + 
                            pow(objectPose.position.z - objectPoseEstimate.position.z, 2));
        double dotProduct = objectPose.orientation.x * objectPoseEstimate.orientation.x + 
                       objectPose.orientation.y * objectPoseEstimate.orientation.y + 
                       objectPose.orientation.z * objectPoseEstimate.orientation.z + 
                       objectPose.orientation.w * objectPoseEstimate.orientation.w;
        double angle = std::acos(2 * pow(dotProduct, 2) - 1);

        printComparison(dist, angle, objectPose, objectPoseEstimate);

        //compare to thresholds
        if (dist <= DIST_THRESHOLD && angle <= ANGLE_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    void myPlanningClass::printComparison(double dist, double angle, geometry_msgs::Pose objectPose, geometry_msgs::Pose objectPoseEstimate)
    {
        std::cout << "Distance: " << dist << std::endl;
        std::cout << "Angle: " << angle << std::endl;
        std::cout << "Object Pose: " << objectPose << std::endl;
        std::cout << "Object Pose Estimate: " << objectPoseEstimate << std::endl;
    }
}