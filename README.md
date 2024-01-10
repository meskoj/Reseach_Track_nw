# Research Track 1: second assignment
This is the program for the second assignment of the course Research Track 1 at the University of Genoa.

## Environment and goal

The environment used in this project is a 2D simulation in Gazebo. The simulated environment features walls and objects, and within it, a robot is programmed to navigate. In addition to Gazebo, also Rviz was used; this last tool is used for ROS visualization, allowing the user to simulate the model and obtaining the robot sensor information. The figure below shows the initial configuration of the robot and the environment in Gazebo.

<p align="center">
    <img src="https://github.com/meskoj/ReseachTrack1_Second_Assignment/assets/145445406/aa841836-3a36-428a-8458-780b13298067">
</p>

The objective is for the robot to reach a user-specified target point. The environment and the algorithm governing the robot's movements have already been implemented and can be referenced from the existing project [documentation](https://github.com/CarmineD8/assignment_2_2023).

## How to run the program
Before starting is necessary to install ROS, to do so follow the instruction on [ROS wiki](https://wiki.ros.org/ROS/Installation) (for the simulation was used the Noetic version).
Then, it is necessary to setup the workspace and create a package called _assignment_2_2023_.
In order to run the script, clone the folder inside the package (ensure not to include the Git project folder within the package) and start the roscore by using the following command:
```bash
$ roscore
```
Finally, to start the simulation is possible to use the following command:
```bash
$ roslaunch assignment_2_2023 simulation.launch
```
(If encountering an error with the command, consider closing the terminal and reopening it).
Following this step, Gazebo and Rviz are expected to render the simulation. Furthermore, each node responsible for implementing the motion algorithm and the server is initiated.

## Code structure and functionality

### Bug0 algorithm for motion planning
The robot employs a simple yet effective algorithm for navigation in unknown environments. The algorithm consists of the following steps:

1. **Move towards goal:** obtained the coordinates of the goal point, the robots starts moving towards it.

2. **Obstacle detection and following:** in case of an obstacle along the path, the robot navigates along its contours until reaching a point where it can resume its path towards the goal.

The algorithm iterates until the robot successfully reaches its goal.

### Action client and interface
The whole program is structured in 3 nodes:
<ol>
  <li> Client: this one implements an Action client using the ActionLib library from ROS. It is used to manage the interaction between the server and the user and to send the goal to the robot. The first part implements an interface to interact with the user and decide which action to perform. The possible actions are: set the target coordinates, cancel the actual target, or exit the program. The script is also responsible to retrieve the information about the actual position and velocity of the robot from the '/odom' topic. Then, it publishes them on the topic '/actual_info', making them accessible for other nodes within the system. Finally, it implements a service to provide the last coordinates set for the target.</li>
  <li> Node B: this script serves as a service client. When executed, it retrieves the most recent coordinates for the target from the server implemented in the Action client. Subsequently, it displays these coordinates on the screen. </li>
  <li>Node C: it implements a service server to relay information about the robot's distance from the target and its average velocity. The script collects real-time data on the robot's position and velocities through the '\robotInfo' topic, where messages are published by the client. Additionally, it retrieves target coordinates using the same service employed in Node B. To formulate its service response, the script computes the distance and the average speed. Notably, the speed calculation incorporates an averaging window, the size of which, is determined by a server parameter, set from the launch file. To achieve this, a First-In-First-Out (FIFO) buffer is created. Velocities from the received messages are continually inserted into this buffer and then the average speed is computed using the specified averaging window size.  </li>
</ol>

## Pseudocode of client node
```cpp
Import necessary libraries and namespaces

Initialize global variables for coordinates, flags, and a mutex for data protection

void interface() {     
    while (programIsNotExited) {
         Display a menu to the user to set target, cancel target, or exit
         Take user input and perform the corresponding action
         If(setTarget){
           ask for x and y coordinates within a valid range}
         If(cancelTarget){
           set the cancel flag to true}
         If(exit){
           shut down the ROS node and set the exitProgram flag to true}
    }
}

void OdometryCallback(msg) {
     Extract position and velocity from the '/odom' topic
}

bool setLastCoordinates(req, res) {
     Set the response with the last x and y coordinates
}

int main() {
     Initialize the ROS node
     Subscribe to the '/odom' topic to get position and velocity
     Advertise on the 'actual_info' topic to publish robot information
     Advertise a service 'last_coordinates' to provide last coordinates
     Wait for the action server to start
     Create a new thread to run the 'interface' function

     while (programIsNotExited) {
        If(cancel){
         check if there is an active goal, if yes, cancel it}
        If(goalNotSent){
           send a new goal to the action server}
         Publish the current position and velocity on the 'actual_info' topic
         If(reachedGoal and notPublished){
           print the result}
    }
}
```
## How to use the program
After having start the whole simulation with the command shown above is possible to set the target from the interface that will show on the terminal. Then, it's also possible to cancel the actual target or exit.
To retrieve the information about target is possible to open another terminal window and write:
```bash
$ roslaunch assignment_2_2023 nodeB
```
Finally, to obtain the actual coordinates and average velocities it's possible to use:
```bash
$ rosservice call robotInfoSrv
```


## Further improvments
The robot demonstrates its capability to navigate within the environment and reach user-selected goal coordinates. However, there is an improvement that can be done. In fact, the current speed of the robot is notably slow, attributed to both a modest actual speed and the absence of an algorithm that allows the robot to reaches the goal finding the shortest path. 
