// #ifndef torque_controller  
// #define torque_controller

#include <include.h>

#define POSITION_MODE 1
#define VELOCITY_MODE 2
#define TORQUE_MODE 3
#define TRAJECTORY_MODE 4

#define SAFETY 0.5

namespace my_torque_controller
{
    class myTorqueControllerClass
    {
        public: myTorqueControllerClass() : FKSolver(sawyerChain), ikSolverVel(sawyerChain), ikSolverPos(sawyerChain, FKSolver, ikSolverVel)
            {
                controllerPub = nh.advertise<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10);
                gravityPub = nh.advertise<std_msgs::Empty>("/robot/limb/right/suppress_gravity_compensation", 10);
                tauExtPub = nh.advertise<std_msgs::Float64MultiArray>("/measurement/tau_ext", 10);
                tauControlPub = nh.advertise<std_msgs::Float64MultiArray>("measurement/tau_control", 10);
                tauModelPub = nh.advertise<std_msgs::Float64MultiArray>("measurement/tau_model", 10);

                jointStateSub = nh.subscribe("/robot/joint_states", 50, &myTorqueControllerClass::jointStateCallback, this);
                jointLimitsSub = nh.subscribe("/robot/joint_limits", 1, &myTorqueControllerClass::jointLimitsCallback, this);
                std::string robot_desc_string;
                nh.param("robot_description", robot_desc_string, std::string());
                
                if (!kdl_parser::treeFromString(robot_desc_string, sawyer))
                {
                    ROS_ERROR("Failed to construct kdl tree");
                }

                joint_names_only_ = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
                
                int N = joint_names_only_.size();

                joint_positions_ = std::vector<double>(N); joint_velocities_ = std::vector<double>(N); joint_efforts_ = std::vector<double>(N);

                qd = std::vector<double>(N); qd_d = std::vector<double>(N); qd_dd = std::vector<double>(N);

                qe = std::vector<double>(N); qe_d = std::vector<double>(N);

                tau_d = std::vector<double>(N); tau_e = std::vector<double>(N); tau_ext = std::vector<double>(N);
                
                M = KDL::JntSpaceInertiaMatrix(N);
                C = KDL::JntArray(N);
                G = KDL::JntArray(N);

                omega = 10; xi = 1; 
                Kp = omega*omega;       // wn^2 - proportional gain
                Kd = 2*omega*xi;        // 2*wn*xi - derivative gain

                checkLoadedTree();
                sawyerChain = tree2chain(sawyer);  
            }
            
            void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
            {  
                joint_names_.clear();
                joint_positions_.clear();
                joint_velocities_.clear();
                joint_efforts_.clear();

                for (int i = 0; i < msg->name.size(); i++)
                {
                    if (msg->name[i] != "head_pan" && msg->name[i] != "torso_t0")
                    {
                        joint_names_.push_back(msg->name[i]);
                        joint_positions_.push_back(msg->position[i]);
                        joint_velocities_.push_back(msg->velocity[i]);
                        joint_efforts_.push_back(msg->effort[i]);
                    }
                }

                for (int i = 0; i < joint_positions_.size(); i++)
                {
                    qe[i] = joint_positions_[i] - qd[i];
                    qe_d[i] = joint_velocities_[i] - qd_d[i];
                    tau_e[i] = joint_efforts_[i] - tau_d[i];
                }
            }

            void jointLimitsCallback(const intera_core_msgs::JointLimits::ConstPtr& msg)
            {
                joint_limits_lower.clear();
                joint_limits_upper.clear();
                velocity_limits.clear();
                accel_limits.clear();
                torque_limits.clear();

                // Copy the data from the message to the corresponding vectors, excluding 'head_pan'
                for (size_t i = 0; i < msg->joint_names.size(); ++i)
                {
                    if (msg->joint_names[i] != "head_pan")
                    {
                        joint_limits_lower.push_back(msg->position_lower[i]);
                        joint_limits_upper.push_back(msg->position_upper[i]);
                        velocity_limits.push_back(msg->velocity[i]);
                        accel_limits.push_back(msg->accel[i]);
                        torque_limits.push_back(msg->effort[i]);
                    }
                }
            }


            std::vector<double> getInitTorques()
            {
                return joint_efforts_;
            }

            void setInitTrajectory()
            {
                for (int i = 0; i < joint_positions_.size(); i++)
                {
                    qd[i] = joint_positions_[i];
                    qd_d[i] = 0.0;
                    qd_dd[i] = 0.0;
                }
                q_init = joint_positions_;
            }

            void setTrajectory(std::vector<double> target_q, std::vector<double> target_qd)
            {
                for (int i = 0; i < qd.size(); i++)
                {
                    qd[i] = target_q[i];
                    qd_d[i] = target_qd[i];
                    qd_dd[i] = 0.0;
                }
            }

            void updateTrajectory(std::vector<double> q_offset, std::vector<double> qd_offset)
            {
                for (int i = 0; i < qd.size(); i++)
                {
                    qd[i] = q_init[i] + q_offset[i];
                    qd_d[i] = 0.0 + qd_offset[i];
                    qd_dd[i] = 0.0;
                }
            }

            void setMatrices(KDL::JntSpaceInertiaMatrix Mass, KDL::JntArray Coriolis, KDL::JntArray Gravity)
            {
                M = Mass;
                C = Coriolis;
                G = Gravity;
            }

            void setDesiredTorque(std::vector<double> torques)
            {
                tau_d = torques;
            }

            void calcExternalTorque(std::vector<double> torque_model)
            {
                for ( int i = 0; i < joint_efforts_.size(); i++)
                {
                    tau_ext[i] = joint_efforts_[i] - torque_model[i];
                }
            }


            /** Publishes an empty message to the gravity compensation topic
             * 
             * @brief publishes an empty message to the gravity compensation topic
             * 
             * @return void
             */
            void gravityCompPublisher();


            /** Publishes a message with a tau array to the specified topic
             * 
             * @brief Publishes a message with a tau array to the specified topic
             *
             * @param ros::Publisher pub Publisher to send message to
             * @param std::vector<double> tau Torque vector to be saved
             * 
             * @return void
             */
            void tauPublisher(ros::Publisher pub, std::vector<double> tau);


            /** Publishes the control mode message to the robot
             * 
             * @brief publishes a joint_command message to the robot to set the controller mode to torque control
             * 
             * @return void
             */
            void torqueControlPublisher();


            /** Solve for inverse dynamics for a given trajectory
            *
            * @brief the moveit plugin defines a trajectory and the KDL plugin uses 
            * the inverse dynamics solver to solve for the torques.
            * 
            * @return std::vector<double> the torques to be send to the robot
            */
            std::vector<double> solveTorques();


            /** Compare torques
             * 
             * @brief compare torques from the inverse dynamics solver with the current torque values
             * throws runtime errors or scales the torques if the torques are not as expected
             * 
             * @param torques the torque input to be checked
             * 
             * @return std::vector<double> scaled torques
             */
            std::vector<double> compareTorques(std::vector<double> torques);


            /** Solves forwards kinematics for the sawyer chain
             * 
             * @brief Uses the sawyerChain object to calculate the end effector frame for a given joint array
             * using forwards kinematics
             * 
             * @param KDL::JntArray q - The joint array that is used to calculate the ende effector position
             * 
             * @return KDL::Frame - The end effector frame for the given position
             */
            KDL::Frame solveFKinematics(KDL::JntArray q);


            /** Solves inverse kinematics for the sawyer chain
             * 
             * @brief Uses the sawyerChain object to calculate the corresponding joint positions for a given end effector frame
             * using inverse kinematics
             * 
             * @param KDL::Frame Frame - The end effector position
             * 
             * @return KDL::JntArray - The corresponding joint positions
             */
            KDL::JntArray solveIKinematics(KDL::Frame Frame);


            /** Checks if the tree is loaded correctly
             * 
             * @brief queries the tree for the base and end effector segments
             * 
             * @return void
             */
            void checkLoadedTree();


            /** Calls the chaindynparam function calculates the torques
             * 
             * @brief calls the chaindynparam function to calculate the Lagrange dynamics matrices
             * and uses these to calculate the input torques for the robot.
             * 
             * @return void
             */
            void computeMatrices();


            /** Converts the KDL tree to a KDL chain
             * 
             * @brief Uses the getChain() function to convert the KDL tree to a KDL chain
             * 
             * @param sawyer the KDL tree of the sawyer robot
             * 
             * @return KDL::Chain the converted KDL chain of the sawyer robot
             */
            KDL::Chain tree2chain(KDL::Tree sawyer);


            /** Converts the std::vector<double> to a KDL::JntArray
             * 
             * @brief creates a new KDL::JntArray and copies the values from the std::vector<double>
             * 
             * @param vec The std::vector<double> to be converted
             * 
             * @return KDL::JntArray the converted KDL::JntArray
             */
            KDL::JntArray vector2JntArray(const std::vector<double> &vec);


            /** Converts the KDL::JntArray to a std::vector<double>
             * 
             * @brief creates a new std::vector<double> and copies the values from the KDL::JntArray
             * 
             * @param array The KDL::JntArray to be converted
             * 
             * @return std::vector<double> the converted std::vector<double>
             */
            std::vector<double> JntArray2vector(const KDL::JntArray &array);


            /** Print joint states
             * 
             * @brief print message to terminal
             * 
             * @return void
             */
            void printStates();


            /** Print system matrices
             * 
             * @brief print message to terminal
             * 
             * @param M the inertia matrix
             * @param C the coriolis matrix
             * @param G the gravity matrix
             * 
             * @return void
             */
            void printMatrices();
            

            /** Print joint command message
             * 
             * @brief print message to terminal
             * 
             * @return void
             */
            void printMessage(intera_core_msgs::JointCommand joint_command_msg);


            /** Print joint information
             * 
             * @brief print current, desired and error of position, velocity and acceleration
             * 
             * @return void
             */
            void printControllerInfo();


            /** Print matrix information
             * 
             * @brief print amount of rows and columns of KDL::JntArray
             * 
             * @return void
             */
            void printMatrixDimensions(const KDL::JntArray& matrix, const std::string& name);

            /** Print matrix information
             * 
             * @brief print amount of rows and columns of KDL::JntSpaceInertiaMatrix
             * 
             * @return void
             */
            void printMatrixDimensions(const KDL::JntSpaceInertiaMatrix& matrix, const std::string& name);

            /** Calculate the sign of a number
             * 
             * @brief Takes in a double and finds the sign of this number. Then returns either 1.0, -1.0 or 0.0
             * 
             * @return double Sign of input
             */
            double getSign(double num);


        private:
            ros::NodeHandle nh;
            ros::Subscriber jointStateSub;
            ros::Publisher controllerPub;
            ros::Publisher gravityPub;
            ros::Subscriber jointLimitsSub;
            ros::Publisher tauExtPub, tauControlPub, tauModelPub;

            std::vector<std::string> joint_names_;
            std::vector<double> joint_positions_, joint_velocities_, joint_efforts_;

            std::vector<double> joint_limits_lower;
            std::vector<double> joint_limits_upper;
            std::vector<double> velocity_limits;
            std::vector<double> accel_limits;
            std::vector<double> torque_limits;

            std::vector<double> qd, qd_d, qd_dd;

            std::vector<double> qe, qe_d;

            std::vector<double> tau_d, tau_e, tau_ext;

            
            std::vector<double> q_init;

            KDL::JntSpaceInertiaMatrix M;
            KDL::JntArray C;
            KDL::JntArray G;

            double Kp, Kd;
            double omega, xi;

            std::vector<std::string> joint_names_only_;

            KDL::Tree sawyer;
            KDL::Chain sawyerChain;

            KDL::ChainFkSolverPos_recursive FKSolver;
            KDL::ChainIkSolverVel_pinv ikSolverVel;
            KDL::ChainIkSolverPos_NR ikSolverPos; 
    };
}

// #endif 
