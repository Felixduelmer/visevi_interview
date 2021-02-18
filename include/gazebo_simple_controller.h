#ifndef GAZEBO_SIMPLE_CONTROLLER_H
#define GAZEBO_SIMPLE_CONTROLLER_H

#include <gazebo/common/Plugin.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <update_timer.h>

namespace gazebo
{
class GazeboSimpleController : public ModelPlugin
{
public:
  GazeboSimpleController();
  virtual ~GazeboSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Publisher wrench_publisher_;
  ros::Publisher link_velocity_publisher_;
  ros::Publisher desired_velocity_publisher_;

  ros::Subscriber _reconfigure_subscriber;

  ros::ServiceServer engage_service_server_;
  ros::ServiceServer shutdown_service_server_;

  // void CallbackQueueThread();
  // boost::mutex lock_;
  // boost::thread callback_queue_thread_;

  geometry_msgs::Twist velocity_command_;
  geometry_msgs::Twist position_command_;
  geometry_msgs::Twist controller_callback_;
  geometry_msgs::Twist real_velocity_;
  void ControllerCallback(const geometry_msgs::TwistConstPtr&);
  void PositionCallback(const geometry_msgs::TwistConstPtr&);
  void VelocityCallback(const geometry_msgs::TwistConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void StateCallback(const nav_msgs::OdometryConstPtr&);

  bool EngageCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool ShutdownCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  ros::Time state_stamp;
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose;
  ignition::math::Vector3d euler, velocity, acceleration, angular_velocity;
#else
  math::Pose pose;
  math::Vector3 euler, velocity, acceleration, angular_velocity, angular_accelaration;
#endif

  std::string link_name_;
  std::string namespace_;
  std::string velocity_topic_;
  std::string position_topic_;
  std::string link_velocity_topic_;
  std::string desired_velocity_topic_;
  std::string imu_topic_;
  std::string state_topic_;
  std::string wrench_topic_;
  std::string reconfigure_topic_;
  double max_force_;
  double max_torque_;

  bool running_;
  bool auto_engage_;

  class PIDController
  {
  public:
    PIDController();
    virtual ~PIDController();
    virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "");

    double gain_p;
    double gain_i;
    double gain_d;
    double time_constant;
    double limit;

    double input;
    double dinput;
    double output;
    double p, i, d;

    double update(double input, double x, double dx, double dt);
    void updateGains(double gain_p, double gain_i, double gain_d, double time_constant);
    void reset();
  };

  struct Controllers
  {
    PIDController roll_vel;
    PIDController pitch_vel;
    PIDController yaw_vel;
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController position_x;
    PIDController position_y;
    PIDController position_z;
  } controllers_;

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d inertia;
#else
  math::Vector3 inertia;
#endif
  double mass;

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d force, torque;
#else
  math::Vector3 force, torque;
#endif

  UpdateTimer controlTimer;
  event::ConnectionPtr updateConnection;
};
}

#endif  // GAZEBO_SIMPLE_CONTROLLER_H
