#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

#include <iostream>
#include <sstream> 

class RobotController{
    public:
        void init_communication(){
            velocity_command_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            drive_to_target_service_ = node_handle_.advertiseService("/ball_chaser/command_robot", &RobotController::drive_to_target_callback, this);
        }
        
        bool drive_to_target_callback(ball_chaser::DriveToTarget::Request& request, ball_chaser::DriveToTarget::Response& response){
            geometry_msgs::Twist velocity_command;
            velocity_command.linear.x = request.linear_x;
            velocity_command.angular.z = request.angular_z;

            velocity_command_publisher_.publish(velocity_command);

            std::string feedback_message = "Command executed: (" + std::to_string(velocity_command.linear.x) + ", " +  std::to_string(velocity_command.angular.z) + ")";  
            response.msg_feedback = feedback_message;
            //ROS_INFO_STREAM(response.msg_feedback);

            return true;
        }
        
    private:
        ros::NodeHandle node_handle_;
        ros::Publisher velocity_command_publisher_;
        ros::ServiceServer drive_to_target_service_;
};

int main(int argc, char** argv){
    // Initialize ROS node
    ros::init(argc, argv, "drive_bot");

    // Create and initialize controller instance
    RobotController controller;
    controller.init_communication();
    
    ros::spin();

    return 0;
}
