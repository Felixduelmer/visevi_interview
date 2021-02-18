#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/JointRequest.h>
#include <std_msgs/Int16.h>
#include <cstdlib>
#include "ros/ros.h"

int force, old_force;

// called quite frequently (~10 Hz)
void cmd_gripCallback(const std_msgs::Int16 &msg)
{
  // apply new force
  force = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_forces");

  ros::NodeHandle n;
  ros::ServiceClient clearForceclient;
  ros::ServiceClient applyForceClient;

  // subscriber to input commmands --> keyboard or myo
  ros::NodeHandle nhandsub;
  ros::Subscriber sub = nhandsub.subscribe("/gripperforce", 10, &cmd_gripCallback);

  // Publish at 10Hz
  ros::Rate loop_rate(10);

  // Set up clients for applying/clearing the efforts
  applyForceClient = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
  clearForceclient = n.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces");

  force = 0;
  old_force = 0;

  // Because we have three different joints at the gripper, we need to define 3 different messages (name is different)
  gazebo_msgs::ApplyJointEffort applyjointeffortjoint[3];
  gazebo_msgs::JointRequest clearJointForces[3];

  clearJointForces[0].request.joint_name = (std::string) "gripper1_gripper2";
  clearJointForces[1].request.joint_name = (std::string) "gripperpart1_gripperpart2";
  clearJointForces[2].request.joint_name = (std::string) "gripperpart2_gripperpart_3";

  applyjointeffortjoint[0].request.joint_name = (std::string) "gripper1_gripper2";
  applyjointeffortjoint[1].request.joint_name = (std::string) "gripperpart1_gripperpart2";
  applyjointeffortjoint[2].request.joint_name = (std::string) "gripperpart2_gripperpart_3";

  // setting jointforces duration to -1 leads to a constant and neverending output of the desired force
  for (int i = 0; i < sizeof(applyjointeffortjoint) / sizeof(applyjointeffortjoint[0]); i++)
  {
    applyjointeffortjoint[i].request.duration = ros::Duration(-1, 0);
  }

  ROS_INFO("Inititalized");
  // clear old force

  while (ros::ok())
  {
    if (force != old_force)
    {
      // as service is cumulative, first clear all active forces on the joints
      for (int i = 0; i < sizeof(clearJointForces) / sizeof(clearJointForces[0]); i++)
      {
        clearForceclient.call(clearJointForces[i]);
      }

      // after this, set new forces to the joints
      for (int i = 0; i < sizeof(applyjointeffortjoint) / sizeof(applyjointeffortjoint[0]); i++)
      {
        applyjointeffortjoint[i].request.start_time = ros::Time::now();
        applyjointeffortjoint[i].request.effort = force;
        applyForceClient.call(applyjointeffortjoint[i]);
      }

      ROS_INFO("Currently you apply: %i", force);
      old_force = force;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
