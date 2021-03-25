#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"  // KEY!!
#include "geometry_msgs/Twist.h"


/*
    Publish:
        /cmd_vel
            geometry_msgs/Twist.msg
            - Vector3 linear (x, y, z)
            - Vector3 angular

    Subscribe:
        /move_base_simple/goal
            geometry_msgs/PoseStamped.msg
            - header
            - pose
        /robot_pose
            geometry_msgs/Twist.msg

        geometry_msgs/Pose.msg:
        - Point
            float64 x
            float64 y
            float64 z
        - Quaternion
            float64 x
            float64 y
            float64 z
            float64 w
*/


ros::Publisher cmd_vel_pub;
ros::Subscriber robot_pose_sub;
ros::Subscriber goal_pose_sub;


const float pi = 3.14159265358979323846;

enum State{MOVING, IDLE};
State state = IDLE; //Initial turtle state/pose.

float rho, alpha, beta;
float goal_x = 0;
float goal_y = 0;
float goal_z = 0;
float goal_theta = 0;
float robot_x = 0;
float robot_y = 0;
float robot_theta = 0;

float error_x;
float error_y;
float error_theta;

void Robot_Pose_Callback(const geometry_msgs::Twist::ConstPtr& msg){
    
    // robot current pose feedback (from sensors measurement)
    robot_x = msg->linear.x;
    robot_y = msg->linear.y;
    robot_theta = msg->angular.z;

    float delta_x = goal_x - robot_x;
    float delta_y = goal_y - robot_y;
    float delta_theta = goal_theta - robot_theta;

    // Convert to polar coordinate.
    rho = sqrt(pow(delta_x,2)+pow(delta_y,2));

    alpha = atan2(delta_y, delta_x) - (robot_theta);
    beta = delta_theta - alpha;

    /* ------ TODO: 4 Quadrant ------ */

    // alpha constraint: -pi ~ pi. 
    if (alpha <= -pi){
        while(alpha <= -pi){
            alpha += 2*pi;
        }
    }
    if (alpha > pi){
        while(alpha > pi)
            alpha -= 2*pi;
    }
    
}



void Goal_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Once receive a command, change state!
    state = MOVING;
    float qx, qy, qz, qw;
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;    
    qw = msg->pose.orientation.w;

    goal_theta = atan2(2*(qw*qz+qx*qy), 1-2*(qz*qz+qy*qy));

}




/* -------- Kinematics ---------- */
const float wheel_r = 0.06; //cm
const float wheel_separaton = 0.24; //cm

const float Max_Wheel_Speed = 2*pi;
const float max_v = 2*wheel_r*Max_Wheel_Speed;
const float max_w = 2*max_v/wheel_separaton; 

/* -------- Gain ---------- */
const float Krho = 0.5;
const float Ka = 2.2; 
const float Kb = -1.2;




int main(int argc, char **argv)
{
    // Initialize the node here
	ros::init(argc, argv, "controller");
    ros::NodeHandle node;

	ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber robot_pose_sub = node.subscribe("/robot_pose", 10, Robot_Pose_Callback);
	ros::Subscriber goal_pose_sub = node.subscribe("/move_base_simple/goal", 10, Goal_Pose_Callback);

    // Set the publish rate here
	ros::Rate rate(100);

    float v, w;
	geometry_msgs::Twist twist;


    // Main Control Loop.

    // After receive a new goal: keep localization task.
	while (ros::ok()) 
	{

        switch(state){
            case MOVING:
                ////////// Main Control Loop! /////////////
                error_x = goal_x - robot_x;
                error_y = goal_y - robot_y;
                error_theta = goal_theta - error_theta;
                ROS_INFO("Error: %f %f %f", error_x, error_y, error_theta);
                if (abs(error_x) < 0.01 && abs(error_y) < 0.01 && abs(error_theta) < 0.01){
                    state = IDLE;
                }
                else{
                    /* --- Control Law --- */
                    v = Krho * rho;
                    w = Ka*alpha + Kb*beta;
                    // ROS_INFO("Robot is MOVING !!!");
                }
            break;
            case IDLE:
                // Stop and listen to new goal.
                v = 0;
                w = 0;
                ROS_INFO("Robot reached the goal and is now IDLE.");
            break;
        }
			
        /* Command Inputs Constraints */
        if ( v > max_v ){
            v = max_v;
        }
        if ( v < -max_v ){
            v = -max_v;
        }        
        if ( w > max_w ){
            w = max_w;
        }
        if ( w < -max_w ){
            w = -max_w;
        }
		// Publish!
        twist.linear.x =  v;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
		twist.angular.z = w;
		cmd_vel_pub.publish(twist); 

        ros::spinOnce();    // Allow processing of incoming messages
        rate.sleep();

    }
}

