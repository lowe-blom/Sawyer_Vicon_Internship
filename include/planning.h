// #ifndef PLANNING_H  
// #define PLANNING_H

#include <include.h>

namespace my_planning
{   
    class myPlanningClass
    {
        public:
            myPlanningClass(): move_group(PLANNING_GROUP)
            {
                ros::NodeHandle node_handle; 

                startPose.position.x = 0.55;
                startPose.position.y = 0.15;
                startPose.position.z = 0.4;
                startPose.orientation.x = 0.738044288105;
                startPose.orientation.y = 0.674450255163;
                startPose.orientation.z = 0.0114540788828;
                startPose.orientation.w = -0.0166218586212;

                releasePose.position.x = 0.5;
                releasePose.position.y = 0.0;
                releasePose.position.z = 0.3;
                releasePose.orientation.x = 0.0;
                releasePose.orientation.y = 1.0;
                releasePose.orientation.z = 0.0;
                releasePose.orientation.w = 0.0;

                gripperState = false;

                viconSub = node_handle.subscribe("/vicon/Sawyer_Object/Sawyer_Object", 10, &myPlanningClass::locationCallback, this);
                viconSub2 = node_handle.subscribe("/vicon/manta1/manta1", 10, &myPlanningClass::mantaCallback, this);
                viconSub3 = node_handle.subscribe("/vicon/clamp/clamp", 10, &myPlanningClass::clampCallback, this);
                gripperSub = node_handle.subscribe("/io/end_effector/stp_022005TP99124/state", 10, &myPlanningClass::gripperCallback, this);
                gripperPub = node_handle.advertise<intera_core_msgs::IOComponentCommand>("/io/end_effector/stp_022005TP99124/command", 10);
                clicksmartPub = node_handle.advertise<intera_core_msgs::IOComponentCommand>("io/end_effector/command", 10);
            }


            void locationCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
            {
                const geometry_msgs::Vector3& translation = msg->transform.translation;
                const geometry_msgs::Quaternion& rotation = msg->transform.rotation;

                objectPose.position.x = translation.x;
                objectPose.position.y = translation.y;
                objectPose.position.z = translation.z;
                objectPose.orientation = rotation;
            }

            void mantaCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
            {
                const geometry_msgs::Vector3& translation = msg->transform.translation;
                const geometry_msgs::Quaternion& rotation = msg->transform.rotation;

                cameraPose.position.x = translation.x;
                cameraPose.position.y = translation.y;
                cameraPose.position.z = translation.z;
                cameraPose.orientation = rotation;
            }

            void clampCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
            {
                const geometry_msgs::Vector3& translation = msg->transform.translation;
                const geometry_msgs::Quaternion& rotation = msg->transform.rotation;

                clampPose.position.x = translation.x;
                clampPose.position.y = translation.y;
                clampPose.position.z = translation.z;
                clampPose.orientation = rotation;
            }

            void gripperCallback(const intera_core_msgs::IODeviceStatus::ConstPtr& msg)
            {
                for (const auto& signal : msg->signals)
                {
                    if (signal.name == "grip_HkZFMKl3Wu")
                    {
                        if (signal.format == "{\"role\":\"output\",\"type\":\"bool\"}")
                        {
                            gripperState = (signal.data == "[true]");
                        } else
                        {
                            ROS_WARN("Unexpected signal format for 'grip_HkZFMKl3Wu'");
                        }
                    }
                }
            }


            geometry_msgs::Pose getStartPose() const
            {
                return startPose;
            }

            geometry_msgs::Pose getObjectPose()
            {
                return objectPose;
            }

            geometry_msgs::Pose getCameraPose()
            {
                return cameraPose;
            }

            geometry_msgs::Pose getClampPose()
            {
                return clampPose;
            }

            bool getGripperState()
            {
                return gripperState;
            }

            ros::Publisher getClicksmartPub() const
            {

                return clicksmartPub;
            }

            ros::Publisher getGripperPub() const
            {
                return gripperPub;
            }

            /** move to a target position.
            *
            * @brief By using the moveit move_group interface the robot calculates a 
            * path to the specified target position.
            * 
            * @return void
            *
            * @param targetPose Target position and orientation
            */
            void goToPoseGoal(geometry_msgs::Pose targetPose);

            /** Compute a trajectory to a target position.
            *
            * @brief By using the moveit move_group interface the robot calculates a 
            * path to the specified target position.
            * 
            * @return void
            *
            * @param targetPose Target position and orientation
            */
            void computeTrajectory(geometry_msgs::Pose targetPose);

            /** move to a target joint state.
            *
            * @brief By using the moveit move_group interface the robot calculates a 
            * path to the specified joint state.
            * 
            * @return void
            *
            */
            void goToJointState();

            /** move along a cartesion path.
            *
            * @brief Specify waypoints for the robot and it will move along this path
            *  by making use of the move_group interface.
            * 
            * @return void
            *
            */
            void cartesionPath();

            /** reset the start state and operational speed.
             * 
             * @brief Resets the start state and operational speed of the robot.
             * 
             * @return void
             * 
             */
            void resetValues();

            /** initialize the gripper by turning on the clicksmart plate
             * 
             * @brief Initializes the gripper by turning on the clicksmart plate
             * 
             * @param pub Publisher to the /io/end_effector/command topic
             * 
             * @return void
            */
            void initGripper(const ros::Publisher& pub);
            
            /** add objects to the planning scene.
             * 
             * @brief Adds objects to the planning scene. The objects are defined in the
             * json file located in the data folder.
             * 
             * @return void
             */
            void addObjects();

            /** make a box.
             * 
             * @brief Makes a box with the specified name and pose.
             * 
             * @param blk_name Name of the box
             * @param pose Pose of the box
             * 
             * @return void
             */
            void makeBox(std::string blk_name, double *pose);

            /** remove objects from the planning scene.
             * 
             * @brief Removes objects from the planning scene. The objects are defined in the
             * json file located in the data folder.
             * 
             * @return void
             */
            void removeObjects();

            /** operate the gripper.
             * 
             * @brief Operates the gripper by publishing to the /io/end_effector/stp_022005TP99124/command
             * topic. The gripper checks the current state and opens if the gripper is closed and 
             * vice versa
             * 
             * @param pub Publisher to the /io/end_effector/stp_022005TP99124/command topic
             * @param gripperState current gripper state
             * 
             * @return void
             */
            void operateGripper(const ros::Publisher& pub, bool gripperState);

            /** synthesize the IOComponentCommand message.
             * 
             * @brief Synthesizes the IOComponentCommand message to be published to the
             * /io/end_effector/stp_022005TP99124/command topic.
             * 
             * @param gripperState current gripper state
             * 
             * @return type: intera_core_msgs::IOComponentCommand --> message
            */
            intera_core_msgs::IOComponentCommand makeMessage(bool gripperState);

            /** print the object pose.
             * 
             * @brief Prints the pose of the object seen by the vicon system.
             * 
             * @param objectPose Pose of the object
             * 
             * @return void
            */
            void printPose(geometry_msgs::Pose objectPose);

            /** rotate the pose along a specified axis by a specified angle.
             * 
             * @brief Rotates the pose along a specified axis by a specified angle.
             * 
             * @param pose Pose to be rotated
             * @param axis Axis to rotate along
             * @param angle Angle to rotate by
             * 
             * @return type: bool -> True if the pose was rotated successfully, false otherwise
            */
            bool rotatePose(geometry_msgs::Pose& pose, char axis, double angle);


            /** go to the object pose.
             * 
             * @brief Moves the robot to the pose of the object seen by the vicon system 
             * while correcting for misalignments.
             * 
             * @param objectPose Pose of the object from vicon
             * @param x x-offset of the object
             * @param y y-offset of the object
             * @param z z-offset of the object
             * 
             * @return void
            */
            void updatePose(geometry_msgs::Pose objectPose, double x, double y, double z);

            /** pick and place the object.
             * 
             * @brief Picks up the object and places it at a specified position.
             * 
             * @param pub Publisher to the /io/end_effector/stp_022005TP99124/command topic
             * @param gripperState current gripper state
             * @param objectPose Pose of the object from vicon
             * 
             * @return void
            */
            void pickAndPlace(const ros::Publisher& pub, geometry_msgs::Pose targetPose);

            /** calculate transformation from cameraframe to viconframe.
             * 
             * @brief calculate the coordinate transformation from the cameraframe to the viconframe.
             * 
             * @param objectPoseEstimate the estimated pose given by Ricardo's neural network. 
             * 
             * @return type: geometry_msgs::Pose -> transformed pose in the vicon frame
            */
            geometry_msgs::Pose poseCam2Vicon(geometry_msgs::Pose objectPoseEstimate);

            /** compare poses
             * 
             * @brief compare the poses of the object from the vicon system to that of 
             * the camera transformed into the world frame.
             * 
             * @param objectPoseVicon pose of the object from the vicon system
             * @param worldPose pose of the object from the camera transformed into the world frame
             * 
             * @return type: bool -> True if the poses are similar, false otherwise
            */
            bool comparePoses(geometry_msgs::Pose objectPoseVicon, geometry_msgs::Pose worldPose);
            
            /** prints the outcome of the comparison
             * 
             * @brief prints the outcome of the comparison between the poses of the object from the vicon system to that of
             * 
             * @param dist distance between the poses
             * @param angle angle between the poses
             * @param objectPoseVicon pose of the object from the vicon system
             * @param worldPose pose of the object from the camera transformed into the world frame
             * 
             * @return void
            */
            void printComparison(double dist, double angle, geometry_msgs::Pose objectPoseVicon, geometry_msgs::Pose worldPose);

        private:
            geometry_msgs::Pose objectPose;
            geometry_msgs::Pose cameraPose;
            geometry_msgs::Pose clampPose;
            bool gripperState;

            ros::Subscriber viconSub;
            ros::Subscriber viconSub2;
            ros::Subscriber viconSub3;
            ros::Subscriber gripperSub;
            ros::Publisher gripperPub;
            ros::Publisher clicksmartPub;
            
            const std::string PLANNING_GROUP = "right_arm";
            const double tau = 2 * M_PI;

            moveit::planning_interface::MoveGroupInterface move_group;
            const robot_state::JointModelGroup* joint_model_group;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::PlanningSceneInterface virtual_world;

            geometry_msgs::Pose startPose;
            geometry_msgs::Pose releasePose;
            geometry_msgs::Point targetPosition1;

    };
}

// #endif 