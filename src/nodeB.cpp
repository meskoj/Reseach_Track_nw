#include <ros/ros.h>
#include <assignment_2_2023/GetCoordinates.h>
#include <assignment_2_2023/PlanningActionGoal.h>

double x = -10;
double y = -10;

bool setLastCoordinates(assignment_2_2023::GetCoordinates::Request& req, assignment_2_2023::GetCoordinates::Response& res) {
    if(x == -10 and y == -10){ 
	// The default value of -10 is arbitrarily chosen to signify that x and y have not been explicitly set yet.
	// These values are outside the environment, so they can't cause problems to the code 
    	ROS_ERROR("Coordinate are not yet set");
    	return false;
    	}
    else{  
	res.x = x;
	res.y = y;
    	return true;}
}

void coordinatesCallback(const assignment_2_2023::PlanningActionGoal::ConstPtr& msg){
    x = msg->goal.target_pose.pose.position.x;
    y = msg->goal.target_pose.pose.position.y;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "nodeB");
    ros::NodeHandle nh;
    
    // Implement node B as service serve
    ros::ServiceServer lastCoordinates = nh.advertiseService("last_coordinates", setLastCoordinates);  
    
    // Subscribe to the goal topic to retrieve the information about the last coordinates set
    ros::Subscriber sub = nh.subscribe("/reaching_goal/goal",10,coordinatesCallback);
    
    ros::spin();
   

    return 0;
}
