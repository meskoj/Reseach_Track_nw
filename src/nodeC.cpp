/**
*\file nodeC.cpp
*\brief Service node that retrieves information about the speed and the distance from the goal
*\author Marco Meschini
*\version 0.2
*\date 29/02/2024
*\details
*
*Subscribes to:<BR>
*	/actual_info
*
*Service: <BR>
*	last_coordinates <BR>
*	robotInfoSrv
*
*Description:
*This node implements a service server to relay information about the robot's distance from the target and its average velocity. The script collects real-time data on the robot's position and velocities through the '\robotInfo' topic, where messages are published by the client. Additionally, it retrieves target coordinates using the service implemented in Node B. To formulate its service response, the script computes the distance and the average speed. Notably, the speed calculation incorporates an averaging window, the size of which, is determined by a server parameter, set from the launch file. To achieve this, a First-In-First-Out (FIFO) buffer is created. Velocities from the received messages are continually inserted into this buffer and then the average speed is computed using the specified averaging window size.
*/
#include <ros/ros.h>
#include <assignment_2_2023/robotInfo.h>
#include <assignment_2_2023/robotInfoSrv.h>
#include <assignment_2_2023/GetCoordinates.h>
#include <cmath>
#include <deque>  

double x, y;
std::deque<double> vel_x_buffer, vel_z_buffer;  // Implents a FIFO queue with deque
int averaging_window;

void robotInfoCallback(const assignment_2_2023::robotInfo::ConstPtr& msg)
{
/**
*\brief Store information about the actual speed
*\param robotInfo_msg
*/
    x = msg->x;
    y = msg->y;

    // Store the velocity values in the buffers
    vel_x_buffer.push_front(msg->vel_x);
    vel_z_buffer.push_front(msg->vel_z);
    
    // Keep the buffer size within the specified averaging window, eliminating oldest velocities
    while (vel_x_buffer.size() > averaging_window) {
        vel_x_buffer.pop_back();
        vel_z_buffer.pop_back();
    }
}

bool computeInfo(assignment_2_2023::robotInfoSrv::Request& req, assignment_2_2023::robotInfoSrv::Response& res)
{
/**
*\brief Compute the actual speed and the distance from the target
*\param robotInfoSrv_request
*\param robotInfoSrv_response
*\return true if there are coordinates set as target, otherwise false
*/
    double target_x, target_y;

    // Retrieve the last target coordinates inserted
    ros::NodeHandle nh;
    ros::service::waitForService("last_coordinates");
    ros::ServiceClient lastCoordinates = nh.serviceClient<assignment_2_2023::GetCoordinates>("last_coordinates");
    assignment_2_2023::GetCoordinates srv;

    if (lastCoordinates.call(srv))
    {
        target_x = srv.response.x;
        target_y = srv.response.y;
    }
    else{
    	return false;
    }

    // Calculate the average velocity within the specified window
    double avg_vel_x = 0.0, avg_vel_z = 0.0;

    // Calculate the average velocity for vel_x
    for (int i = 0; i < vel_x_buffer.size(); ++i) {
        avg_vel_x += vel_x_buffer[i];
    }
    avg_vel_x /= vel_x_buffer.size();

    // Calculate the average velocity for vel_z
    for (int i = 0; i < vel_z_buffer.size(); ++i) {
        avg_vel_z += vel_z_buffer[i];
    }
    avg_vel_z /= vel_z_buffer.size();

    // Compute the speed and distance, giving them as response to the service
    double distance = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));

    res.distance = distance;
    res.vel_x = avg_vel_x;
    res.vel_z = avg_vel_z;
    return true;
}

int main(int argc, char** argv)
{s
    ros::init(argc, argv, "robot_info_service");
    
    ros::NodeHandle nh;
    
    ros::Subscriber sub_pose = nh.subscribe("/actual_info", 1, robotInfoCallback);
    
    ros::ServiceServer robotInfoServer = nh.advertiseService("robotInfoSrv", computeInfo);

    // Retrieve the information about the dimension of the averaging_window in parameter server
    ros::param::get("averaging_window", averaging_window);
    
    ros::spin();
}
