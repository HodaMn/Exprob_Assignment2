#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <string>
#include <iostream>
#include <cstdlib>

namespace KCL_rosplan {

class MyActionInterface {
public:
    MyActionInterface(ros::NodeHandle &nh);

    void markerPointCB(const geometry_msgs::Point::ConstPtr &msg);
    void markerNumCB(const std_msgs::Int32::ConstPtr &msg);
    bool concreteCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);

private:
    void handleGoToAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg, move_base_msgs::MoveBaseGoal &goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac);
    void handleDetectAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);
    void handleCheckMarkersAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg, move_base_msgs::MoveBaseGoal &goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac);
    void setGoalPosition(move_base_msgs::MoveBaseGoal &goal, double x, double y);
    void updateMarkerData(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);
    void logMarkerData();

    ros::NodeHandle nh;
    ros::Publisher Pub1;
    ros::Subscriber MarkerCenter_subscriber;
    ros::Subscriber MarkerNum_subscriber;

    double marker_center_x;
    double marker_center_y;
    double camera_width;
    bool flag;
    double error;
    double threshold;
    double marker_id;
    int final_location_x;
    int final_location_y;
    double least_marker_id;
    std::string last_location;
};

MyActionInterface::MyActionInterface(ros::NodeHandle &nh)
    : marker_center_x(0.0),
      marker_center_y(0.0),
      camera_width(320.0),
      flag(true),
      error(0.0),
      threshold(18.0),
      marker_id(0.0),
      final_location_x(0),
      final_location_y(0),
      least_marker_id(200.0),
      last_location("wp4") {

    Pub1 = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MarkerCenter_subscriber = nh.subscribe("/marker/center_loc", 10, &MyActionInterface::markerPointCB, this);
    MarkerNum_subscriber = nh.subscribe("/marker/id_number", 10, &MyActionInterface::markerNumCB, this);
}

void MyActionInterface::markerPointCB(const geometry_msgs::Point::ConstPtr &msg) {
    marker_center_x = msg->x;
    marker_center_y = msg->y;
}

void MyActionInterface::markerNumCB(const std_msgs::Int32::ConstPtr &msg) {
    marker_id = msg->data;
}

bool MyActionInterface::concreteCB(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    ac.waitForServer();

    if (msg->name == "go_to") {
        handleGoToAction(msg, goal, ac);
    } else if (msg->name == "detect") {
        handleDetectAction(msg);
    } else if (msg->name == "check_markers") {
        handleCheckMarkersAction(msg, goal, ac);
    }

    ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
    return true;
}

void MyActionInterface::handleGoToAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg, move_base_msgs::MoveBaseGoal &goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac) {
    std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.orientation.w = 1.0;

    if (msg->parameters[2].value == "wp1") {
        setGoalPosition(goal, -3.0, -8.0);
    } else if (msg->parameters[2].value == "wp2") {
        setGoalPosition(goal, 6.0, 2.0);
    } else if (msg->parameters[2].value == "wp3") {
        setGoalPosition(goal, 7.0, -5.0);
    } else if (msg->parameters[2].value == "wp4") {
        setGoalPosition(goal, 0.0, 2.75);
    } else if (msg->parameters[2].value == "wp0") {
        setGoalPosition(goal, -7.0, 1.5);
    }

    ac.sendGoal(goal);
    ac.waitForResult();
}

void MyActionInterface::handleDetectAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
    std::cout << "Rotating to find the marker in Place" << std::endl;

    while (flag) {
        error = std::abs(marker_center_x - camera_width);

        geometry_msgs::Twist twist;
        twist.angular.z = 0.5;
        Pub1.publish(twist);

        if (error < threshold) {
            flag = false;

            std::cout << "The marker with the following ID is found: x = " << marker_id << std::endl;

            if (marker_id < least_marker_id) {
                updateMarkerData(msg);
            } else {
                logMarkerData();
            }

            twist.angular.z = 0.0;
            Pub1.publish(twist);
        }
    }

    flag = true;
}

void MyActionInterface::handleCheckMarkersAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg, move_base_msgs::MoveBaseGoal &goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac) {
    nh.getParam("last_loc", last_location);
    nh.getParam("least_id", least_marker_id);

    std::cout << "Going back to " << last_location << " with marker ID " << least_marker_id << std::endl;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.orientation.w = 1.0;

    if (last_location == "wp1") {
        setGoalPosition(goal, -3.0, -8.0);
    } else if (last_location == "wp2") {
        setGoalPosition(goal, 6.0, 2.0);
    } else if (last_location == "wp3") {
        setGoalPosition(goal, 7.0, -5.0);
    } else if (last_location == "wp4") {
        setGoalPosition(goal, 0.0, 2.75);
    } else if (last_location == "wp0") {
        setGoalPosition(goal, -7.0, 1.5);
    }

    ac.sendGoal(goal);
    ac.waitForResult();
}

void MyActionInterface::setGoalPosition(move_base_msgs::MoveBaseGoal &goal, double x, double y) {
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
}

void MyActionInterface::updateMarkerData(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
    least_marker_id = marker_id;
    last_location = msg->parameters[1].value;

    nh.setParam("last_loc", last_location);
    nh.setParam("least_id", least_marker_id);

    logMarkerData();
}

void MyActionInterface::logMarkerData() {
    std::cout << "Updated least marker ID: " << least_marker_id
              << " at goal location x = " << last_location << std::endl;
}

} // namespace KCL_rosplan

int main(int argc, char **argv) {
    ros::init(argc, argv, "assignment_plan_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::MyActionInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}

