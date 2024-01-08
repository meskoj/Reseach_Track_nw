#include <ros/ros.h>
#include <assignment_2_2023/GetCoordinates.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "service_client");
    ros::NodeHandle nh;

    // Wait for the 'last coordinates' service to become available
    ros::service::waitForService("last_coordinates");

    // Create a service client for the 'last coordinates' service
    ros::ServiceClient lastCoordinates = nh.serviceClient<assignment_2_2023::GetCoordinates>("last_coordinates");

    // Create a GetCoordinates service response
    assignment_2_2023::GetCoordinates srv;

    // Call the 'last_coordinates' service
    if (lastCoordinates.call(srv))
    {
    	// The values -10 are arbitrary, they were set on that value because outside the environment, to modify them go in the client script
    	if(srv.response.x == -10 and srv.response.y == -10){ 
    		ROS_ERROR("Coordinate are not yet set");
    	}
    	else{    		
    	ROS_INFO("Last x coordinate inserted: %f",srv.response.x);
    	ROS_INFO("Last y coordinate inserted: %f",srv.response.y);
    	}
    }
    else
    {
        // Print an error message if the service call failed
        ROS_ERROR("Failed to call GetCoordinates service.");
        return 1;
    }

    return 0;
}
