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
//            0       1       2
enum State{MOVING, TURNING, IDLE};
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


    if(robot_theta > pi){
        while(robot_theta > pi)
            robot_theta -= 2*pi;
    }
    if(robot_theta <= -pi){
        while(robot_theta <= 2*pi)
            robot_theta += 2*pi;
    }

    float delta_x = goal_x - robot_x;
    float delta_y = goal_y - robot_y;
    float delta_theta = goal_theta - robot_theta;

    // Convert to polar coordinate.
    rho = sqrt(pow(delta_x,2)+pow(delta_y,2));


        // % xc : robot_x.
        // % xi : initial_x.


        // % 1st - quardrum
        float x_c = goal_x;
        float y_c = goal_y;
        float angle_c = goal_theta;

        float x_i = robot_x;
        float y_i = robot_y;
        float angle = robot_theta;

        
        float x_err = error_x;
        float y_err = error_y;


        if (x_c - x_i >= 0 && y_c - y_i >= 0){
            alpha = (atan2(y_err,x_err) - angle);
            beta = angle_c - atan2(y_err,x_err);
        }
        // % 2nd - quardrum
        if (x_c - x_i <= 0 && y_c - y_i >= 0){
            alpha = (atan2(y_err,x_err) - angle);
            beta = angle_c + (2*pi - atan2(y_err,x_err));
        }
        // % 3rd quardrum
        if (x_c - x_i <=0 && y_c - y_i <=0){
            alpha = pi + atan2(y_err,x_err) - angle + pi;
            beta = -(pi + atan2(y_err,x_err) - angle_c) + pi;
        }

        // % 4th quardrum
        if (x_c - x_i >=0 && y_c - y_i <=0){
            alpha = pi + atan2(y_err,x_err) - angle + pi;
            beta = -(pi + atan2(y_err,x_err) - angle_c) + pi;
        }

    // alpha = atan2(delta_y, delta_x) - (robot_theta);

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


    if (beta <= -pi){
        while(beta <= -pi){
            beta += 2*pi;
        }
    }
    if (beta > pi){
        while(beta > pi)
            beta -= 2*pi;
    }

    // beta = delta_theta - alpha;

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
    
    // if(goal_theta <0){
    //     while(goal_theta <0)
    //         goal_theta += 2*pi;
    // }
    // if(goal_theta > 2*pi){
    //     while(goal_theta>2*pi)
    //         goal_theta -= 2*pi;
    // }

}




/* -------- Kinematics ---------- */
const float wheel_r = 0.06; //cm
const float wheel_separaton = 0.24; //cm

const float Max_Wheel_Speed = 2*pi;
const float max_v = 2*wheel_r*Max_Wheel_Speed;
const float max_w = 2*max_v/wheel_separaton; 

/* -------- Gain ---------- */
// const float Krho = 0.5;
// const float Ka = 2.2; 
// const float Kb = -0.6;

// Claude
const float Krho = 2;
const float Ka = 8;
const float Kb = -1.5;
const float Kp = 0.5;

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

        error_x = goal_x - robot_x;
        error_y = goal_y - robot_y;
        error_theta = goal_theta - robot_theta;

        ROS_INFO("State: %d, Goal theta: %.2f, Robot Theta: %.2f", state, goal_theta, robot_theta);
        ROS_INFO("Error: %.3f %.3f %.3f", error_x, error_y, error_theta);

        switch(state){
            case MOVING:
                ////////// Main Control Loop! /////////////
                if (abs(error_x) < 0.01 && abs(error_y) < 0.01 && abs(error_theta) < 0.1){
                    state = IDLE;
                }
                else if(abs(error_x) < 0.01 && abs(error_y) < 0.01){
                    state = TURNING;
                }
                else{
                    /* --- Control Law --- */
                    v = Krho * rho;
                    w = Ka*alpha + Kb*beta;
                    // ROS_INFO("Robot is MOVING !!!");
                }
            break;
            case TURNING:
 
                if ( abs(error_theta) < 0.1 || abs(robot_theta + 2*pi)<0.1 || abs(error_theta - 2*pi) < 0.1){
                    state = IDLE;
                }
                else{
                    /* --- Control Law --- */
                    v = 0;
                    w = Kp * error_theta;
                }

            break;
            case IDLE:
                // Stop and listen to new goal.
                // reset error signals.
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

