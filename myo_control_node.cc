#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <prosthesis_v7/ReconfigureConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/convert.h"
#include "ros_myo/MyoPose.h"
#include <std_msgs/Int16.h>
#include <math.h>

double euler[3], euler_orig[3], euler_offset[3], euler_old[3];
geometry_msgs::Twist twist;
bool grasp, first;
tf2::Quaternion q_orig, q_rot;
bool reset;
std_msgs::Int16 applied_force;


//define the orientation of the prosthesis
//handle the singularity that comes along when dealing witht quaternion to euler angle tranformations
void poseCallback(const geometry_msgs::PoseStamped &pose) {

    tf2::Quaternion q_new(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    tf2::Matrix3x3 m(q_new);

    //get roll, pitch, yaw
    m.getRPY(euler[0], euler[1], euler[2]);

    //set resetting values and initial values
    if(reset){
        for(int i = 0; i<3; i++){
            euler_orig[i] = euler[i];
            euler_old[i] = euler[i];
            euler_offset[i] = 0;
        }
        reset = false;
    }
    
    //check if jump has happened
    //if negative jump add positive offset otherwise negative
    for(int i = 0; i<3; i++){
        if(fabs(euler[i]-euler_old[i]) > 1){
            euler[i]-euler_old[i]<0 ? euler_offset[i]+=2*M_PI : euler_offset[i]-=2*M_PI;
        }
        euler_old[i] = euler[i];
    }

    //set twist angles, euler angles are received in "wrong order"
    twist.angular.x = -(euler[2]-euler_orig[2]+euler_offset[2]);
    twist.angular.y = (euler[1]-euler_orig[1]+euler_offset[1]);
    twist.angular.z = -(euler[0]-euler_orig[0]+euler_offset[0]);

    ROS_INFO("You're sending r: %f p: %f y: %f values", (twist.angular.x*360)/(2*M_PI), (twist.angular.y*360)/(2*M_PI), (twist.angular.z*360)/(2*M_PI));

}

//check the lateral movement of the whole prosthesis
void lateralCallback(const geometry_msgs::Twist &input){
    twist.linear.x += input.linear.x*0.01;
    twist.linear.y += input.linear.y*0.01;
    twist.linear.z += input.linear.z*0.01;
}

//Check whether user wants to grasp or not
void fistCallback(const ros_myo::MyoPose &pose){
    //if hand is a fist
    if(2 == pose.pose){
        grasp = true;
    }
    //if hand is in rest
    if(1 == pose.pose){
        grasp = false;
    }
}


//subscribe to all input topics and publish the state of the prosthesis
int main(int argc, char **argv) {
    ros::init(argc, argv, "myo_control_node");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_pos", 10);

    ros::NodeHandle nhandpubfist;
    ros::Publisher pubfist = nhandpubfist.advertise<std_msgs::Int16>("/gripperforce", 10);

    ros::NodeHandle nhandsublat;
    ros::Subscriber sublat = nhandsublat.subscribe("/desired_lateral_cmd_pos", 10, &lateralCallback);

    ros::NodeHandle nhandsubpose;
    ros::Subscriber subpose = nhandsubpose.subscribe("/myo_raw/pose", 10, &poseCallback);

    ros::NodeHandle nhandsubfist;
    ros::Subscriber fist_contro = nhandsubfist.subscribe("/myo_raw/myo_gest", 10, &fistCallback);

    ros::Rate loop_rate(10);

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    grasp = false;
    reset = true;

    ROS_INFO("Spinning node");

    while(ros::ok()){
        pub.publish(twist);
        //if true set value to positive to grasp otherwise open gripper
        grasp ? applied_force.data = 30 : applied_force.data = -30;
        pubfist.publish(applied_force);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}