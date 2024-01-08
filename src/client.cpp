#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/GetCoordinates.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2023/robotInfo.h>
#include <assignment_2_2023/robotInfoSrv.h>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <mutex>

using namespace std;

// Define global variables to store the information 
double x = -10; // These values are assigned outside the acceptable range to trigger the while
double y = -10;
double actual_x, actual_y, actual_linear_x, actual_angular_z;
bool cancel = false;
bool goalSent = true;
bool exitProgram = false;
bool published = false;

// Mutex for synchronization to protect shared data
mutex dataMutex;

void interface() {
    while (!exitProgram) {
        int sel;
	ros::Duration(2.0).sleep();
        cout << "Select what to do:\n";
        cout << "1: Set target\n";
        cout << "2: Cancel target\n";
        cout << "3: Exit\n";

        cin >> sel;

        unique_lock<mutex> lock(dataMutex);

        if (sel == 1) {
            // Everytime we enter inside this if, set the coordinates outside the range to trigger the while
            x = -10;
            y = -10;
            // Select the desired coordinates
            while (x < -9 or x > 9){
                cout << "Please enter a value for the desired x coordinate (between -9 and 9): ";
                cin >> x;
            }
            while(y < -9 or y > 9){   
                cout << "Please enter a value for the desired y coordinate (between -9 and 9): ";
                cin >> y;
            }
            goalSent = false;
        } 
        else if (sel == 2) {
            cancel = true;
        } 
        else if (sel == 3) {
            ROS_INFO("Killing ROS node...");
            ros::requestShutdown();
            exitProgram = true;
        } 
        else {
            cout << "Selection not valid. Insert again your choice\n";
        }
	// Unlock the mutex
        lock.unlock(); 
    }
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Get position and velocity from the msg in the /odom topic
    actual_x = msg->pose.pose.position.x;
    actual_y = msg->pose.pose.position.y;
    actual_linear_x = msg->twist.twist.linear.x;
    actual_angular_z = msg->twist.twist.angular.z;
}

bool setLastCoordinates(assignment_2_2023::GetCoordinates::Request& req, assignment_2_2023::GetCoordinates::Response& res) {
    res.x = x;
    res.y = y;
    return true;
}

int main(int argc, char** argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;

    // Subscribe to odom topic to retrieve information about position and velocities
    ros::Subscriber sub_pose = nh.subscribe("/odom", 1, OdometryCallback);

    // Publish on the topic actual info a robotInfo msg
    ros::Publisher pub_pose = nh.advertise<assignment_2_2023::robotInfo>("actual_info", 1);

    // Implement server for node B
    ros::ServiceServer lastCoordinates = nh.advertiseService("last_coordinates", setLastCoordinates);

    actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac("/reaching_goal", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    
    // Send goal to the action server
    assignment_2_2023::PlanningGoal goal;

    // Use thread to create a non-blocking interface
    thread t1(interface);

    while (exitProgram == false) {
        unique_lock<mutex> lock(dataMutex);

        if (cancel == true) {
            // If there is an active goal to cancel, cancel it
            if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
                ac.cancelGoal();
                ROS_INFO("The target has been cancelled successfully!!");
                cancel = false;
            } 
            else {
                // Display an error message if there is no active goal
                ROS_ERROR("Error. There's no target to cancel.");
                cancel = false;
            }
        }
	// If the goal is not sent yet, send it
        if (goalSent == false) {
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;
            ac.sendGoal(goal);
            goalSent = true;
            published = false;
        }
        
	// Publish data on the topic
        assignment_2_2023::robotInfo msg;
        msg.x = actual_x;
        msg.y = actual_y;
        msg.vel_x = actual_linear_x;
        msg.vel_z = actual_angular_z;

        pub_pose.publish(msg);

        lock.unlock();
        ros::spinOnce();
        
        // If the goal has been reached and this result has not been published yet, publish it
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED and published == false){
            ROS_INFO("The goal has been reached");
            published = true;
        }
    }

    t1.join();

    return 0;
}
