#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ImageProcessor{
    public:
        void init_communications(){
            drive_to_target_client_ = node_handle_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
            camera_subscriber_ = node_handle_.subscribe("/camera/rgb/image_raw", 10, &ImageProcessor::process_image_callback, this);
        }

        void drive_robot(float lin_x, float ang_z){
            ball_chaser::DriveToTarget srv;
            srv.request.linear_x = lin_x;
            srv.request.angular_z = ang_z;

            if(!drive_to_target_client_.call(srv))
                ROS_WARN("Failed to call service command_robot!");
        }

        void process_image_callback(const sensor_msgs::Image img){
            /** Image Structure:
             *   1D Vector of length: img.height*img.step
             *   img.step = (# of channels)*img.width
             *   In RGB, # of channels = 3
             *   [pixel1_r, pixel1_g, pixel1_b, pixel2_r, pixel2_g ...]
             **/
            bool found_ball = false;

            // Divide the image in three sections: LEFT, CENTER, RIGHT
            int segment_length = img.step / 3;

            // Search for a white pixel (RGB: (255, 255, 255)) (from a white ball)
            for(int i = 0; i < img.height*img.step; i+=3){
                if(img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255){
                    found_ball = true;
                    int horizontal_index = i % img.step;
                    if(horizontal_index <=  segment_length){
                        ROS_INFO("TURN LEFT!");
                        drive_robot(0.0, -0.2); 
                        break;
                    }else if(horizontal_index <= 2*segment_length){
                        ROS_INFO("MOVE FORWARD!");
                        drive_robot(0.1, 0.0);
                        break;
                    }else{
                        ROS_INFO("TURN RIGHT!");
                        drive_robot(0.0, 0.2);
                        break;
                    } 
                }
            }

            if(!found_ball){
                ROS_INFO("HALT!");
                drive_robot(0.0, 0.0);
            }
        }

    private:
        ros::NodeHandle node_handle_;
        ros::Subscriber camera_subscriber_;
        ros::ServiceClient drive_to_target_client_;
};


int main(int argc, char** argv){
    // Initialize ROS node
    ros::init(argc, argv, "process_image");

    // Create and initialize image processor instance
    ImageProcessor processor;
    processor.init_communications();
 
    ros::spin();

    return 0;
}

