/**
*\file client.cpp
*\brief Client server for the simulation
*\author Marco Meschini
*\version 0.2
*\date 29/02/2024
*\details
*
*Subscribes to:<BR>
*	/odom <BR>
*Publish to:<BR>
*	actual_info <BR>
*Description:<Br>
*this node implements an Action client using the ActionLib library from ROS. It is used to manage the interaction between the server and the user and to send the goal to the robot. The first part implements an interface to interact with the user and decide which action to perform. The possible actions are: set the target coordinates, cancel the actual target, or exit the program. The script is also responsible to retrieve the information about the actual position and velocity of the robot from the '/odom' topic. Then, it publishes them on the topic '/actual_info', making them accessible for other nodes within the system. Finally, it implements a service to provide the last coordinates set for the target.
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2023/robotInfo.h>
#include <assignment_2_2023/robotInfoSrv.h>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <mutex>

using namespace std;

// Define global variables to store the information 
double x = -10; ///< This value is assigned outside the acceptable range to trigger the while
double y = -10; ///< This value is assigned outside the acceptable range to trigger the while
double actual_x, actual_y, actual_linear_x, actual_angular_z;
bool cancel = false;
bool goalSent = true;
bool exitProgram = false;
bool published = false;

// Mutex for synchronization to protect shared data
mutex dataMutex;

void interface() {
/**
*\brief Interface for the program
*
*This function is used to create a graphical interface for the client and provide information about the program
*/
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
/**
*\brief Get position and velocity from the msg in the /odom topic
*\param nav_msgs
*/
    // Get position and velocity from the msg in the /odom topic
    actual_x = msg->pose.pose.position.x;
    actual_y = msg->pose.pose.position.y;
    actual_linear_x = msg->twist.twist.linear.x;
    actual_angular_z = msg->twist.twist.angular.z;
}

int main(int argc, char** argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;

    // Subscribe to odom topic to retrieve information about position and velocities
    ros::Subscriber sub_pose = nh.subscribe("/odom", 1, OdometryCallback);

    // Publish on the topic actual info a robotInfo msg
    ros::Publisher pub_pose = nh.advertise<assignment_2_2023::robotInfo>("actual_info", 1);

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
