#include <include.h>
#include <torque_controller.h>
#include <planning.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_interfacing");
    my_torque_controller::myTorqueControllerClass my_torque_controller_;

    ros::Duration(1).sleep();

    ros::AsyncSpinner spinner(2);
    spinner.start();

    if(argc != 2)
    {
        ROS_INFO(" ");
        ROS_INFO("\tUsage:");\
        ROS_INFO(" ");
        ROS_INFO("\trosrun planning run  n");
        return 1;
    }

    int selection = atoi(argv[1]);

    ros::Duration(2).sleep();
    ros::Rate rate(20);

    switch(selection)
    {
        case 1: // Torque control
            my_torque_controller_.setInitTrajectory();
            my_torque_controller_.setDesiredTorque(my_torque_controller_.getInitTorques());
            ros::Duration(1.0).sleep();
            my_torque_controller_.printStates();
            my_torque_controller_.printControllerInfo();
            my_torque_controller_.computeMatrices();
            ros::Duration(2).sleep();
            while (ros::ok())
            {
                // my_torque_controller_.updateTrajectory(target_q, target_qd);
                my_torque_controller_.printControllerInfo();
                my_torque_controller_.gravityCompPublisher();
                my_torque_controller_.torqueControlPublisher();
                rate.sleep();
            }
            break;
        case 2: // Test kinematics
            KDL::JntArray q = my_torque_controller_.vector2JntArray(my_torque_controller_.getInitPos());
            std::cout << "Joint Array (q): ";
            for (size_t i = 0; i < q.rows(); ++i) {
                std::cout << q(i);
                if (i < q.rows() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << std::endl;
            KDL::Frame Frame = my_torque_controller_.solveFKinematics(q);
            std::cout << "Translation Vector: " << Frame.p.x() << ", " << Frame.p.y() << ", " << Frame.p.z() << std::endl;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    std::cout << Frame.M(i, j) << " ";
                }
                std::cout << std::endl;
            }
            KDL::JntArray q_ik = my_torque_controller_.solveIKinematics(Frame);
            std::cout << "Joint Array (q_ik): ";
            for (size_t i = 0; i < q_ik.rows(); ++i) {
                std::cout << q_ik(i);
                if (i < q_ik.rows() - 1) {
                    std::cout << ", ";
                }
            }
            break;
    }

    spinner.stop();
    std::cout << "Waiting for shutdown" << std::endl;
    ros::waitForShutdown();
    return 0;
}