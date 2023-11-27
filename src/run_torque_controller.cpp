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
        case 2: 
            
            break;
    }

    spinner.stop();
    std::cout << "Waiting for shutdown" << std::endl;
    ros::waitForShutdown();
    return 0;
}