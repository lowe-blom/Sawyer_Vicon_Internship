#include <planning.h>
#include <include.h>
#include <cameraLib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_interfacing");
    my_planning::myPlanningClass my_planning_;
    my_cam::myCamClass my_cam_;

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

    my_planning_.resetValues();
    ros::Duration(1).sleep();

    switch(selection)
    {
        case 1: // initialise
            my_planning_.initGripper(my_planning_.getClicksmartPub());
            my_planning_.addObjects();
            break;
        case 2: // go to start pose
            my_planning_.goToPoseGoal(my_planning_.getStartPose());
            break;
        case 3: // Pick and place state (make sure to initialise with case 1 first)
            my_planning_.pickAndPlace(my_planning_.getGripperPub(), my_planning_.getObjectPose());
            break;
        case 4: // debugging state
            my_planning_.poseCam2Vicon(my_planning_.getStartPose());
            break;
        case 5: // remove objects
            my_planning_.removeObjects();
            break;
        case 6: //  operate gripper
            ros::Duration(5).sleep();
            my_planning_.operateGripper(my_planning_.getGripperPub(), my_planning_.getGripperState());
            break;
    }

    //my_planning_.goToJointState();
    //my_planning_.cartesionPath();

    my_cam_.shutdownVimba(my_cam_.getSystem());
    spinner.stop();
    std::cout << "Waiting for shutdown" << std::endl;
    ros::waitForShutdown();
    return 0;
}