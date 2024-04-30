/**
*\file nodeB.cpp
*\brief Service node to retrieves the last target sent by the user
*\author Marco Meschini
*\version 0.2
*\date 29/02/2024
*\details
*
*Subscribes to:<BR>
*	/reaching_goal/goal
*
*Service: <BR>
*	last_coordinates <BR>
*Description:
*This node serves as a service client. When executed, it retrieves the most recent coordinates for the target from the server implemented in the Action client. Subsequently, it displays these coordinates on the screen.
*/
#include <ros/ros.h>
#include <assignment_2_2023/GetCoordinates.h>
#include <assignment_2_2023/PlanningActionGoal.h>

double x = -10; ///< This value is assigned outside the acceptable range to trigger the while
double y = -10; ///< This value is assigned outside the acceptable range to trigger the while

bool setLastCoordinates(assignment_2_2023::GetCoordinates::Request& req, assignment_2_2023::GetCoordinates::Response& res) {
/**
*\brief This function print the information about the target coordinates
*\param service request 
*\param service response
*\return true if there is a target, otherwise false
*/
    if(x == -10 and y == -10){ 
	// The default value of -10 is arbitrarily chosen to signify that x and y have not been explicitly set yet.
	// These values are outside the environment, so they can't cause problems to the code 
    	ROS_ERROR("Coordinates are not yet set");
    	return false;
    	}
    else{  
	res.x = x;
	res.y = y;
    	return true;}
}

void coordinatesCallback(const assignment_2_2023::PlanningActionGoal::ConstPtr& msg){
/**
*\brief Retrieve the information about the goal using a message
*\param goal_msg
*/
    x = msg->goal.target_pose.pose.position.x;
    y = msg->goal.target_pose.pose.position.y;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "nodeB");
    ros::NodeHandle nh;
    
    // Implement node B as service server
    ros::ServiceServer lastCoordinates = nh.advertiseService("last_coordinates", setLastCoordinates);  
    
    // Subscribe to the goal topic to retrieve the information about the last coordinates set
    ros::Subscriber sub = nh.subscribe("/reaching_goal/goal",10,coordinatesCallback);
    
    ros::spin();
   

    return 0;
}
