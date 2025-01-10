// Include the ROS header for using ROS functionalities.
#include <ros/ros.h>

// Include the ROSPlan Action Interface header.
#include "rosplan_action_interface/RPActionInterface.h"

// Include necessary message types for publishing and subscribing.
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <string>

// Define a namespace for the class, which is a common practice in C++ to organize code.
namespace KCL_rosplan {

// Declare the MyActionInterface class, inheriting from RPActionInterface.
class MyActionInterface : public RPActionInterface {

private:
    // Private Methods
    void initializeSubscribersAndPublishers();
    void initializeMemberVariables();

    // Node handle for interacting with ROS.
    ros::NodeHandle nh;

    // ROS publisher for sending velocity commands.
    ros::Publisher Pub1;

    // ROS subscribers for receiving data.
    ros::Subscriber MarkerCenter_subscriber;
    ros::Subscriber MarkerNum_subscriber;

    // Various private member variables for internal use.
    double camera_width;
    double marker_center_x;
    double marker_center_y;
    double marker_id;
    bool flag;
    double error;
    double threshold;
    double final_location_x;
    double final_location_y;
    double least_marker_id;
    std::string last_location;
    

public:
    // Constructor declaration.
    MyActionInterface(ros::NodeHandle &nh);

    // Method to process action dispatch messages from ROSPlan.
    bool concreteCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

    // CallBack method for marker point data.
    void markerPointCB(const geometry_msgs::Point::ConstPtr& msg);
    
    void markerNumCB(const std_msgs::Int32::ConstPtr& msg)  ; 
};

} 


