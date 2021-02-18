#include <gazebo_simple_controller.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/Wrench.h>
#include <cmath>

namespace gazebo
{
GazeboSimpleController::GazeboSimpleController()
{
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboSimpleController::~GazeboSimpleController()
{
#if (GAZEBO_MAJOR_VERSION < 8)
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
#endif
  updateConnection.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

//////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();
  link = _model->GetLink();
  link_name_ = link->GetName();

  // default parameters
  namespace_.clear();
  velocity_topic_ = "cmd_vel";
  position_topic_ = "cmd_pos";
  reconfigure_topic_ = "reconfigure_node";
  link_velocity_topic_ = "link_velocity_topic";
  desired_velocity_topic_ = "desired_velocity_topic";
  imu_topic_.clear();
  state_topic_.clear();
  wrench_topic_ = "wrench_out";
  max_force_ = -1;
  max_torque_ = -1;
  auto_engage_ = true;

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  if (_sdf->HasElement("topicName"))
    velocity_topic_ = _sdf->GetElement("topicName")->Get<std::string>();
  if (_sdf->HasElement("posComName"))
    position_topic_ = _sdf->GetElement("topicName")->Get<std::string>();
  if (_sdf->HasElement("imuTopic"))
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();
  if (_sdf->HasElement("stateTopic"))
    state_topic_ = _sdf->GetElement("stateTopic")->Get<std::string>();
  if (_sdf->HasElement("wrenchTopic"))
    wrench_topic_ = _sdf->GetElement("wrenchTopic")->Get<std::string>();
  if (_sdf->HasElement("maxForce"))
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();
  if (_sdf->HasElement("maxTorque"))
    max_torque_ = _sdf->GetElement("maxTorque")->Get<double>();
  if (_sdf->HasElement("autoEngage"))
    auto_engage_ = _sdf->GetElement("autoEngage")->Get<bool>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue())
  {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("gazebo plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // configure controllers
  controllers_.roll_vel.Load(_sdf, "roll_vel");
  controllers_.pitch_vel.Load(_sdf, "pitch_vel");
  controllers_.yaw_vel.Load(_sdf, "yaw_vel");
  controllers_.roll.Load(_sdf, "roll");
  controllers_.pitch.Load(_sdf, "pitch");
  controllers_.yaw.Load(_sdf, "yaw");
  controllers_.velocity_x.Load(_sdf, "velocityXY");
  controllers_.velocity_y.Load(_sdf, "velocityXY");
  controllers_.velocity_z.Load(_sdf, "velocityZ");
  controllers_.position_x.Load(_sdf, "positionx");
  controllers_.position_y.Load(_sdf, "positionx");
  controllers_.position_z.Load(_sdf, "positionz");

// Get inertia and mass of body
#if (GAZEBO_MAJOR_VERSION >= 8)
  inertia = link->GetInertial()->PrincipalMoments();
  mass = link->GetInertial()->Mass();
#else
  inertia = link->GetInertial()->GetPrincipalMoments();
  mass = link->GetInertial()->GetMass();
#endif

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  ros::NodeHandle param_handle(*node_handle_, "controller");

  // subscribe command
  param_handle.getParam("velocity_topic", velocity_topic_);
  if (!velocity_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        velocity_topic_, 1, boost::bind(&GazeboSimpleController::VelocityCallback, this, _1), ros::VoidPtr(),
        &callback_queue_);
    velocity_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command_position
  param_handle.getParam("position_topic", position_topic_);
  if (!position_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        position_topic_, 1, boost::bind(&GazeboSimpleController::PositionCallback, this, _1), ros::VoidPtr(),
        &callback_queue_);
    position_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe imu
  param_handle.getParam("imu_topic", imu_topic_);
  if (!imu_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
        imu_topic_, 1, boost::bind(&GazeboSimpleController::ImuCallback, this, _1), ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("simple_controller",
                   "Using imu information on topic %s as source of orientation and angular velocity.",
                   imu_topic_.c_str());
  }

  // subscribe state
  param_handle.getParam("state_topic", state_topic_);
  if (!state_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
        state_topic_, 1, boost::bind(&GazeboSimpleController::StateCallback, this, _1), ros::VoidPtr(),
        &callback_queue_);
    state_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("simple_controller", "Using state information on topic %s as source of state information.",
                   state_topic_.c_str());
  }

  // subscribe reconfigure_node
  param_handle.getParam("reconfigure_topic", reconfigure_topic_);
  if (!reconfigure_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        reconfigure_topic_, 1, boost::bind(&GazeboSimpleController::ControllerCallback, this, _1), ros::VoidPtr(),
        &callback_queue_);
    _reconfigure_subscriber = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("simple_controller", "Using %s as source for reconfigure information", state_topic_.c_str());
  }

  // advertise wrench
  param_handle.getParam("wrench_topic", wrench_topic_);
  if (!wrench_topic_.empty())
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<geometry_msgs::Wrench>(
        wrench_topic_, 10, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(),
        &callback_queue_);
    wrench_publisher_ = node_handle_->advertise(ops);
  }

  param_handle.getParam("link_velocity_topic", link_velocity_topic_);
  if (!link_velocity_topic_.empty())
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<geometry_msgs::Twist>(
        link_velocity_topic_, 10, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(),
        &callback_queue_);
    link_velocity_publisher_ = node_handle_->advertise(ops);
  }

  param_handle.getParam("desired_velocity_topic", desired_velocity_topic_);
  if (!desired_velocity_topic_.empty())
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<geometry_msgs::Twist>(
        desired_velocity_topic_, 10, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
        ros::VoidConstPtr(), &callback_queue_);
    desired_velocity_publisher_ = node_handle_->advertise(ops);
  }

  // engage/shutdown service servers
  {
    ros::AdvertiseServiceOptions ops = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        "engage", boost::bind(&GazeboSimpleController::EngageCallback, this, _1, _2), ros::VoidConstPtr(),
        &callback_queue_);
    engage_service_server_ = node_handle_->advertiseService(ops);

    ops = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        "shutdown", boost::bind(&GazeboSimpleController::ShutdownCallback, this, _1, _2), ros::VoidConstPtr(),
        &callback_queue_);
    shutdown_service_server_ = node_handle_->advertiseService(ops);
  }

  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  controlTimer.Load(world, _sdf);
  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboSimpleController::Update, this));
}

//////////////////////////////////////////////////////////////////////////////
// Callbacks

void GazeboSimpleController::ControllerCallback(const geometry_msgs::TwistConstPtr &controller_setting)
{
  controller_callback_ = *controller_setting;
  // ROS_INFO_NAMED("simple_controller", "Reconfigure Request: %f, proportional: %f, differntial %f, integral: %f,
  // time_constant: %f", controller_callback_.linear.x, controller_callback_.linear.y, controller_callback_.linear.z,
  // controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 1)
    controllers_.velocity_x.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                        controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 2)
    controllers_.position_x.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                        controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 3)
    controllers_.velocity_y.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                        controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 4)
    controllers_.position_y.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                        controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 5)
    controllers_.velocity_z.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                        controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 6)
    controllers_.position_z.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                        controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 7)
    controllers_.roll_vel.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                      controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 8)
    controllers_.roll.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                  controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 9)
    controllers_.pitch_vel.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                       controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 10)
    controllers_.pitch.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                   controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 11)
    controllers_.yaw_vel.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                     controller_callback_.angular.x, controller_callback_.angular.y);
  if (controller_callback_.linear.x == 12)
    controllers_.yaw.updateGains(controller_callback_.linear.y, controller_callback_.linear.z,
                                 controller_callback_.angular.x, controller_callback_.angular.y);
}

void GazeboSimpleController::PositionCallback(const geometry_msgs::TwistConstPtr &position)
{
  position_command_ = *position;
}

void GazeboSimpleController::VelocityCallback(const geometry_msgs::TwistConstPtr &velocity)
{
  velocity_command_ = *velocity;
}

void GazeboSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr &imu)
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(
      ignition::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
#else
  pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.rot.GetAsEuler();
  angular_velocity =
      pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
#endif
}

void GazeboSimpleController::StateCallback(const nav_msgs::OdometryConstPtr &state)
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d velocity1(velocity);
#else
  math::Vector3 velocity1(velocity);
#endif

  if (imu_topic_.empty())
  {
#if (GAZEBO_MAJOR_VERSION >= 8)
    pose.Pos().Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
    pose.Rot().Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y,
                   state->pose.pose.orientation.z);
    euler = pose.Rot().Euler();
#else
    pose.pos.Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
    pose.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y,
                 state->pose.pose.orientation.z);
    euler = pose.rot.GetAsEuler();
#endif
    angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
  }

  velocity.Set(state->twist.twist.linear.x, state->twist.twist.linear.y, state->twist.twist.linear.z);

  // calculate acceleration
  double dt = !state_stamp.isZero() ? (state->header.stamp - state_stamp).toSec() : 0.0;
  state_stamp = state->header.stamp;
  if (dt > 0.0)
  {
    acceleration = (velocity - velocity1) / dt;
  }
  else
  {
    acceleration.Set();
  }
}

bool GazeboSimpleController::EngageCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  ROS_INFO_NAMED("simple_controller", "Engaging motors!");
  running_ = true;
  return true;
}

bool GazeboSimpleController::ShutdownCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  ROS_INFO_NAMED("simple_controller", "Shutting down motors!");
  running_ = false;
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboSimpleController::Update()
{
  // Get new commands/state
  callback_queue_.callAvailable();

  double dt;
  if (controlTimer.update(dt) && dt > 0.0)
  {
// Get Pose/Orientation from Gazebo (if no state subscriber is active)
#if (GAZEBO_MAJOR_VERSION >= 8)
    if (imu_topic_.empty())
    {
      pose = link->WorldPose();
      angular_velocity = link->WorldAngularVel();
      euler = pose.Rot().Euler();
    }
    if (state_topic_.empty())
    {
      acceleration = (link->WorldLinearVel() - velocity) / dt;
      velocity = link->WorldLinearVel();
    }
#else
    if (imu_topic_.empty())
    {
      pose = link->GetWorldPose();
      angular_velocity = link->GetWorldAngularVel();
      angular_accelaration = link->GetWorldAngularAccel();
      euler = pose.rot.GetAsEuler();
      real_velocity_.angular.x = angular_velocity.x;
      real_velocity_.angular.y = angular_velocity.y;
      real_velocity_.angular.z = angular_velocity.z;
    }
    if (state_topic_.empty())
    {
      acceleration = (link->GetWorldLinearVel() - velocity) / dt;
      velocity = link->GetWorldLinearVel();
      real_velocity_.linear.x = velocity.x;
      real_velocity_.linear.y = velocity.y;
      real_velocity_.linear.z = velocity.z;
    }
#endif

    // Auto engage/shutdown
    if (auto_engage_)
    {
      if (!running_ && position_command_.linear.z > 0.1)
      {
        running_ = true;
        ROS_INFO_NAMED("simple_controller", "Engaging motors!");
      }
    }

//  static Time lastDebug;
//  if ((world->GetSimTime() - lastDebug).Double() > 0.5) {
//    ROS_DEBUG_STREAM_NAMED("simple_controller", "Velocity:         gazebo = [" << link->GetWorldLinearVel()   << "],
//    state = [" << velocity << "]");
//    ROS_DEBUG_STREAM_NAMED("simple_controller", "Acceleration:     gazebo = [" << link->GetWorldLinearAccel() << "],
//    state = [" << acceleration << "]");
//    ROS_DEBUG_STREAM_NAMED("simple_controller", "Angular Velocity: gazebo = [" << link->GetWorldAngularVel() << "],
//    state = [" << angular_velocity << "]");
//    lastDebug = world->GetSimTime();
//  }

// Get gravity
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d gravity_body = pose.Rot().RotateVector(world->Gravity());
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body);  // Get gravity
#else
    math::Vector3 gravity_body = pose.rot.RotateVector(world->GetPhysicsEngine()->GetGravity());
    double gravity = gravity_body.GetLength();
    double load_factor = gravity * gravity / world->GetPhysicsEngine()->GetGravity().Dot(gravity_body);  // Get gravity
#endif

// Rotate vectors to coordinate frames relevant for control
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Quaterniond heading_quaternion(cos(euler.Z() / 2), 0, 0, sin(euler.Z() / 2));
    ignition::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    ignition::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);
#else
    math::Quaternion heading_quaternion(cos(euler.z / 2), 0, 0, sin(euler.z / 2));
    math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);
#endif

    // update controllers
    force.Set(0.0, 0.0, 0.0);
    torque.Set(0.0, 0.0, 0.0);
    if (running_)
    {
#if (GAZEBO_MAJOR_VERSION >= 8)
      double pitch_command =
          controllers_.velocity_x.update(velocity_command_.linear.x, velocity_xy.X(), acceleration_xy.X(), dt) /
          gravity;
      double roll_command =
          -controllers_.velocity_y.update(velocity_command_.linear.y, velocity_xy.Y(), acceleration_xy.Y(), dt) /
          gravity;
      torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
      torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
      torque.Z() = inertia.Z() * controllers_.yaw.update(velocity_command_.angular.z, angular_velocity.Z(), 0, dt);
      force.Z() =
          mass * (controllers_.velocity_z.update(velocity_command_.linear.z, velocity.Z(), acceleration.Z(), dt) +
                  load_factor * gravity);
      if (max_force_ > 0.0 && force.Z() > max_force_)
        force.Z() = max_force_;
      if (force.Z() < 0.0)
        force.Z() = 0.0;
#else
      // changed paramaters
      // ROS_INFO_NAMED("simple_controller", "timestep: %f, world coordinates: x: %f y: %f z: %f r: %f p %f y %f", dt,
      // pose.pos.x, pose.pos.y, pose.pos.z, euler.x, euler.y, euler.z);
      velocity_command_.linear.x =
          controllers_.position_x.update(position_command_.linear.x, pose.pos.x, velocity.x, dt);
      velocity_command_.linear.y =
          controllers_.position_y.update(position_command_.linear.y, pose.pos.y, velocity.y, dt);
      velocity_command_.linear.z =
          controllers_.position_z.update(position_command_.linear.z, pose.pos.z, velocity.z, dt);
      force.x = mass * controllers_.velocity_x.update(velocity_command_.linear.x, velocity.x, acceleration.x, dt);
      force.y = mass * controllers_.velocity_y.update(velocity_command_.linear.y, velocity.y, acceleration.y, dt);
      force.z = mass * (controllers_.velocity_z.update(velocity_command_.linear.z, velocity.z, acceleration.z, dt) +
                        load_factor * gravity);
      velocity_command_.angular.x =
          controllers_.roll.update(position_command_.angular.x, euler.x, angular_velocity.x, dt);
      velocity_command_.angular.y =
          controllers_.pitch.update(position_command_.angular.y, euler.y, angular_velocity.y, dt);
      velocity_command_.angular.z =
          controllers_.yaw.update(position_command_.angular.z, euler.z, angular_velocity.z, dt);
      torque.x =
          inertia.x *
          controllers_.roll_vel.update(velocity_command_.angular.x, angular_velocity.x, angular_accelaration.x, dt);
      torque.y =
          inertia.y *
          controllers_.pitch_vel.update(velocity_command_.angular.y, angular_velocity.y, angular_accelaration.y, dt);
      torque.z =
          inertia.z *
          controllers_.yaw_vel.update(velocity_command_.angular.z, angular_velocity.z, angular_accelaration.z, dt);

      // double pitch_command = controllers_.velocity_x.update(velocity_command_.linear.x, velocity_xy.x,
      // acceleration_xy.x, dt) / gravity;
      // double roll_command = -controllers_.velocity_y.update(velocity_command_.linear.y, velocity_xy.y,
      // acceleration_xy.y, dt) / gravity;

      // torque.x = inertia.x *  controllers_.roll.update(-velocity_command_.linear.y/gravity, euler.x,
      // angular_velocity_body.x, dt);
      // torque.y = inertia.y *  controllers_.pitch.update(velocity_command_.linear.x/gravity, euler.y,
      // angular_velocity_body.y, dt);
      if (max_force_ > 0.0 && fabs(force.z) + 10 > max_force_)
        force.z = (force.z > max_force_) ? max_force_ + 10 : -max_force_ - 10;
      if (max_force_ > 0.0 && fabs(force.x) > max_force_)
        force.x = (force.x > max_force_) ? max_force_ : -max_force_;
      if (max_force_ > 0.0 && fabs(force.y) > max_force_)
        force.y = (force.y > max_force_) ? max_force_ : -max_force_;
      if (max_torque_ > 0.0 && fabs(torque.x) > max_torque_)
        torque.x = (torque.x > max_force_) ? max_torque_ : -max_torque_;
      if (max_torque_ > 0.0 && fabs(torque.y) > max_torque_)
        torque.y = (torque.y > max_force_) ? max_torque_ : -max_torque_;
      if (max_torque_ > 0.0 && fabs(torque.z) > max_torque_)
        torque.z = (torque.z > max_force_) ? max_torque_ : -max_torque_;

// ROS_INFO_NAMED("simple_controller", "Forces applied: x: %f y: %f z: %f r: %f p %f y %f", force.x, force.y, force.z,
// torque.x, torque.y, torque.z);

// old parameters
// double pitch_command = controllers_.velocity_x.update(velocity_command_.linear.x, velocity_xy.x, acceleration_xy.x,
// dt) / gravity;
// double roll_command = -controllers_.velocity_y.update(velocity_command_.linear.y, velocity_xy.y, acceleration_xy.y,
// dt) / gravity;
// torque.x = inertia.x * controllers_.roll.update(roll_command, euler.x, angular_velocity_body.x, dt);
// torque.y = inertia.y * controllers_.pitch.update(pitch_command, euler.y, angular_velocity_body.y, dt);
// // torque.x = inertia.x *  controllers_.roll.update(-velocity_command_.linear.y/gravity, euler.x,
// angular_velocity_body.x, dt);
// // torque.y = inertia.y *  controllers_.pitch.update(velocity_command_.linear.x/gravity, euler.y,
// angular_velocity_body.y, dt);
// torque.z = inertia.z * controllers_.yaw.update(velocity_command_.angular.z, angular_velocity.z, 0, dt);
// force.z = mass * (controllers_.velocity_z.update(velocity_command_.linear.z, velocity.z, acceleration.z, dt) +
// load_factor * gravity);
// if (max_force_ > 0.0 && force.z > max_force_)
//   force.z = max_force_;
// if (force.z < 0.0)
//   force.z = 0.0;
#endif
    }
    else
    {
      controllers_.roll.reset();
      controllers_.pitch.reset();
      controllers_.yaw.reset();
      controllers_.roll_vel.reset();
      controllers_.pitch_vel.reset();
      controllers_.yaw_vel.reset();
      controllers_.velocity_x.reset();
      controllers_.velocity_y.reset();
      controllers_.velocity_z.reset();
    }

    //  static double lastDebugOutput = 0.0;
    //  if (last_time.Double() - lastDebugOutput > 0.1) {
    //    ROS_DEBUG_NAMED("simple_controller", "Velocity = [%g %g %g], Acceleration = [%g %g %g]", velocity.x,
    //    velocity.y, velocity.z, acceleration.x, acceleration.y, acceleration.z);
    //    ROS_DEBUG_NAMED("simple_controller", "Command: linear = [%g %g %g], angular = [%g %g %g], roll/pitch = [%g
    //    %g]", velocity_command_.linear.x, velocity_command_.linear.y, velocity_command_.linear.z,
    //    velocity_command_.angular.x*180/M_PI, velocity_command_.angular.y*180/M_PI,
    //    velocity_command_.angular.z*180/M_PI, roll_command*180/M_PI, pitch_command*180/M_PI);
    //    ROS_DEBUG_NAMED("simple_controller", "Mass: %g kg, Inertia: [%g %g %g], Load: %g g", mass, inertia.x,
    //    inertia.y, inertia.z, load_factor);
    //    ROS_DEBUG_NAMED("simple_controller", "Force: [%g %g %g], Torque: [%g %g %g]", force.x, force.y, force.z,
    //    torque.x, torque.y, torque.z);
    //    lastDebugOutput = last_time.Double();
    //  }

    // Publish wrench
    if (wrench_publisher_)
    {
      geometry_msgs::Wrench wrench;
#if (GAZEBO_MAJOR_VERSION >= 8)
      wrench.force.x = force.X();
      wrench.force.y = force.Y();
      wrench.force.z = force.Z();
      wrench.torque.x = torque.X();
      wrench.torque.y = torque.Y();
      wrench.torque.z = torque.Z();
#else
      wrench.force.x = force.x;
      wrench.force.y = force.y;
      wrench.force.z = force.z;
      wrench.torque.x = torque.x;
      wrench.torque.y = torque.y;
      wrench.torque.z = torque.z;
#endif
      wrench_publisher_.publish(wrench);
    }
    link_velocity_publisher_.publish(real_velocity_);
    desired_velocity_publisher_.publish(velocity_command_);
  }

  // set force and torque in gazebo
  link->AddForce(force);
#if (GAZEBO_MAJOR_VERSION >= 8)
  link->AddRelativeTorque(torque - link->GetInertial()->CoG().Cross(force));
#else
  link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().Cross(force));
#endif
}

//     // set force and torque in gazebo
//     link->AddRelativeForce(force);
// #if (GAZEBO_MAJOR_VERSION >= 8)
//     link->AddRelativeTorque(torque - link->GetInertial()->CoG().Cross(force));
// #else
//     link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().Cross(force));
// #endif
//   }

//////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboSimpleController::Reset()
{
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  force.Set();
  torque.Set();

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();

  running_ = false;
}

//////////////////////////////////////////////////////////////////////////////
// PID controller implementation
GazeboSimpleController::PIDController::PIDController()
{
}

GazeboSimpleController::PIDController::~PIDController()
{
}

void GazeboSimpleController::PIDController::Load(sdf::ElementPtr _sdf, const std::string &prefix)
{
  gain_p = 0.0;
  gain_d = 0.0;
  gain_i = 0.0;
  time_constant = 0.0;
  limit = -1.0;

  if (!_sdf)
    return;
  // _sdf->PrintDescription(_sdf->GetName());
  if (_sdf->HasElement(prefix + "ProportionalGain"))
    gain_p = _sdf->GetElement(prefix + "ProportionalGain")->Get<double>();
  if (_sdf->HasElement(prefix + "DifferentialGain"))
    gain_d = _sdf->GetElement(prefix + "DifferentialGain")->Get<double>();
  if (_sdf->HasElement(prefix + "IntegralGain"))
    gain_i = _sdf->GetElement(prefix + "IntegralGain")->Get<double>();
  if (_sdf->HasElement(prefix + "TimeConstant"))
    time_constant = _sdf->GetElement(prefix + "TimeConstant")->Get<double>();
  if (_sdf->HasElement(prefix + "Limit"))
    limit = _sdf->GetElement(prefix + "Limit")->Get<double>();
}

void GazeboSimpleController::PIDController::updateGains(double gain_p_new, double gain_d_new, double gain_i_new,
                                                        double time_constant_new)
{
  gain_p = gain_p_new;
  gain_d = gain_d_new;
  gain_i = gain_i_new;
  time_constant = time_constant_new;
}

double GazeboSimpleController::PIDController::update(double new_input, double x, double dx, double dt)
{
  // limit command
  if (limit > 0.0 && fabs(new_input) > limit)
    new_input = (new_input < 0 ? -1.0 : 1.0) * limit;

  // filter command
  if (dt + time_constant > 0.0)
  {
    dinput = (new_input - input) / (dt + time_constant);
    input = (dt * new_input + time_constant * input) / (dt + time_constant);
  }

  // update proportional, differential and integral errors
  p = input - x;
  d = dinput - dx;
  i = i + dt * p;

  // update control output
  output = gain_p * p + gain_d * d + gain_i * i;
  return output;
}

void GazeboSimpleController::PIDController::reset()
{
  input = dinput = 0;
  p = i = d = output = 0;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboSimpleController)

}  // namespace gazebo
