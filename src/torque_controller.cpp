#include <torque_controller.h>

namespace my_torque_controller
{
    void myTorqueControllerClass::gravityCompPublisher()
    {
        std_msgs::Empty empty_msg;
        gravityPub.publish(empty_msg);
        std::cout << "Gravity compensation empty message published" << std::endl;
    }

    void myTorqueControllerClass::tauPublisher(ros::Publisher pub, tau)
    {
        std_msgs::Float64MultiArray msg;
        msg.data = tau;

        // Publish the message
        pub.publish(msg);
    }

    void myTorqueControllerClass::torqueControlPublisher()
    {
        intera_core_msgs::JointCommand joint_command_msg;
        std::vector<double> torques = solveTorques();
        std::vector<double> scaledTorques = compareTorques(torques);

        joint_command_msg.header.stamp = ros::Time::now();

        joint_command_msg.names = joint_names_only_;
        joint_command_msg.mode = TORQUE_MODE;
        joint_command_msg.effort = scaledTorques;

        printMessage(joint_command_msg);
        controllerPub.publish(joint_command_msg);
    }

    std::vector<double> myTorqueControllerClass::solveTorques()
    {
        std::vector<double> torques, torques_model;
        KDL::JntArray tau_model(qe.size()), tau(qe.size()), qd_dd_(qe.size()), qe_d_(qe.size()), qe_(qe.size()), q_(qe.size()), control(qe.size());

        qd_dd_ = vector2JntArray(qd_dd);
        qe_d_ = vector2JntArray(qe_d);
        qe_ = vector2JntArray(qe);
        q_ = vector2JntArray(joint_positions_);

        computeMatrices();

        control.data = qd_dd_.data - Kd*qe_d_.data - Kp*qe_.data;
        tau.data = M.data*control.data + C.data + G.data;

        tau_model.data = M.data*qd_dd_.data + C.data + G.data;

        torques = JntArray2vector(tau);
        torques_model = JntArray2vector(tau_model);

        setDesiredTorque(torques);
        calcExternalTorque(torques_model);

        return torques;
    }

    std::vector<double> myTorqueControllerClass::compareTorques(std::vector<double> torques)
    {
        bool size = (torques.size() == joint_names_only_.size());
        bool amplitude = true;

        for (int i = 0; i < torques.size(); i++)
        {
            std::cout << "Torque input right_j" << i << ": " << torques[i] << std::endl;
            std::cout << "Current effort right_j" << i << ": " << joint_efforts_[i] << std::endl;

            if (std::abs(torques[i]) > torque_limits[i]*SAFETY)
            {
                std::cout << "Torque output outside of expected range. Scaling torque " << i << std::endl;
                torques[i] = torque_limits[i]*SAFETY*getSign(torques[i]);
                amplitude = false;
            }
        }

        std::cout << "Torque input size: " << size << std::endl;
        std::cout << "Torque input amplitude: " << amplitude << std::endl;
        std::cout << "------------------------------------------" << std::endl;

        if(!size) throw std::runtime_error("Torque input does not match the number of joints");
        if(!amplitude) throw std::runtime_error("Torque output outside of expected range");
        
        return torques;
    }

    void myTorqueControllerClass::printStates()
    {
        for (int i = 0; i < joint_names_.size(); i++)
        {
            std::cout << "Joint name: " << joint_names_[i] << std::endl;
            std::cout << "Joint position: " << joint_positions_[i] << std::endl;
            std::cout << "Joint velocity: " << joint_velocities_[i] << std::endl;
            std::cout << "Joint effort: " << joint_efforts_[i] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }
    }
    
    void myTorqueControllerClass::checkLoadedTree()
    {
        KDL::SegmentMap::const_iterator root_segment_it = sawyer.getRootSegment();

        // Check if the root segment exists
        if (root_segment_it == sawyer.getSegments().end())
        {
            ROS_ERROR("Root segment not found in the loaded KDL tree.");
            return;
        }

        // Check if the end effector segment exists
        KDL::SegmentMap::const_iterator end_effector_segment_it = sawyer.getSegment("right_hand");
        if (end_effector_segment_it == sawyer.getSegments().end())
        {
            ROS_WARN("End effector segment not found in the loaded KDL tree.");
        }

        ROS_INFO("KDL tree checked successfully.");
    }

    KDL::Chain myTorqueControllerClass::tree2chain(KDL::Tree sawyer)
    {
        KDL::Chain sawyer_chain;
        if (!sawyer.getChain("base", "right_hand", sawyer_chain))
        {
            ROS_ERROR("Failed to get KDL chain from tree.");
        }
        return sawyer_chain;
    }

    void myTorqueControllerClass::computeMatrices()
    {
        KDL::ChainDynParam dyn_param(sawyerChain, KDL::Vector(0, 0, -9.81));
        int N = sawyerChain.getNrOfJoints();
        
        // Use current position and velocity to calculate torques (for verification)
        KDL::JntArray q = vector2JntArray(joint_positions_);
        KDL::JntArray q_dot = vector2JntArray(joint_velocities_);

        KDL::JntSpaceInertiaMatrix Mass(N);
        KDL::JntArray Coriolis(N);
        KDL::JntArray Gravity(N);

        // Compute the mass matrix
        dyn_param.JntToMass(q, Mass);

        // Compute the Coriolis torques
        dyn_param.JntToCoriolis(q, q_dot, Coriolis);

        // Compute the gravity torques
        dyn_param.JntToGravity(q, Gravity);

        // Print the matrices
        setMatrices(Mass, Coriolis, Gravity);
        printMatrices();
    }


    KDL::JntArray myTorqueControllerClass::vector2JntArray(const std::vector<double> &vec)
    {
        KDL::JntArray jnt_array(vec.size());

        for (unsigned int i = 0; i < vec.size(); ++i)
        {
            jnt_array(i) = vec[i];
        }

        return jnt_array;
    }


    std::vector<double> myTorqueControllerClass::JntArray2vector(const KDL::JntArray &array)
    {
        std::vector<double> vector;

        for (int i = 0; i < array.data.size(); i++)
        {
            vector.push_back(array.data[i]);
        }

        return vector;
    }

    void myTorqueControllerClass::printMatrices()
    {
        // Print the mass matrix
        std::cout << "" << std::endl;
        std::cout << "Mass Matrix:" << std::endl;
        for (unsigned int i = 0; i < M.rows(); ++i)
        {
            for (unsigned int j = 0; j < M.columns(); ++j)
            {
                std::cout << M(i, j) << "\t";
            }
            std::cout << std::endl;
        }

        // Print the Coriolis torques
        std::cout << "" << std::endl;
        std::cout << "Size of C: " << C.rows() << std::endl;
        std::cout << "Coriolis Torques:" << std::endl;
        for (unsigned int i = 0; i < C.rows(); ++i)
        {
            std::cout << C(i) << "\t";
        }
        std::cout << std::endl;

        // Print the gravity torques
        std::cout << "" << std::endl;
        std::cout << "Size of G: " << G.rows() << std::endl;
        std::cout << "Gravity Torques:" << std::endl;
        for (unsigned int i = 0; i < G.rows(); ++i)
        {
            std::cout << G(i) << "\t";
        }
        std::cout << std::endl;
        std::cout << "-----------------------------------------" << std::endl;
    }

    void myTorqueControllerClass::printMessage(intera_core_msgs::JointCommand joint_command_msg)
    {
        std::cout << "Joint command message:" << std::endl;
        std::cout << "Mode: " << joint_command_msg.mode << std::endl;
        std::cout << "Timestamp: " << joint_command_msg.header.stamp << std::endl;
        for (int i = 0; i < joint_command_msg.names.size(); i++)
        {
            std::cout << "Name: " << joint_command_msg.names[i] << std::endl;
            std::cout << "Effort: " << joint_command_msg.effort[i] << std::endl;
        }
        std::cout << "------------------------------------------" << std::endl;
    }


    void myTorqueControllerClass::printControllerInfo()
    {
        for ( int i = 0; i < qd.size(); i++ )
        {
            std::cout << "Current position joint_j" << i << " q: " << joint_positions_[i] << std::endl;
            std::cout << "Desired position joint_j" << i << " qd: " << qd[i] << std::endl;
            std::cout << "Error of position joint_j" << i << " qe: " << qe[i] << std::endl;
            std::cout << "------------------------------------------" << std::endl;

            std::cout << "Current velocity joint_j" << i << " q_d: " << joint_velocities_[i] << std::endl;
            std::cout << "Desired velocity joint_j" << i << " qd_d: " << qd_d[i] << std::endl;
            std::cout << "Error of velocity joint_j" << i << " qe_d: " << qe_d[i] << std::endl;
            std::cout << "------------------------------------------" << std::endl;

            std::cout << "Desired acceleration joint_j" << i << " qd_dd: " << qd_dd[i] << std::endl;
            std::cout << "------------------------------------------" << std::endl;

            std::cout << "Current torque joint_j" << i << " tau: " << joint_efforts_[i] << std::endl;
            std::cout << "Desired torque joint_j" << i << " tau_d: " << tau_d[i] << std::endl;
            std::cout << "Error of torque joint_j" << i << " tau_e: " << tau_e[i] << std::endl;
            std::cout << "------------------------------------------" << std::endl;
        }
    }

    void myTorqueControllerClass::printMatrixDimensions(const KDL::JntArray& matrix, const std::string& name) 
    {
        std::cout << "Matrix '" << name << "' has rows: " << matrix.rows() << ", columns: " << matrix.columns() << std::endl;
    }

    void myTorqueControllerClass::printMatrixDimensions(const KDL::JntSpaceInertiaMatrix& matrix, const std::string& name) 
    {
        std::cout << "Matrix '" << name << "' has rows: " << matrix.rows() << ", columns: " << matrix.columns() << std::endl;
    }

    double getSign(double num) 
    {
        if (num > 0) {
            return 1.0;
        } else if (num < 0) {
            return -1.0;
        } else {
            return 0.0;
        }
    }
}
