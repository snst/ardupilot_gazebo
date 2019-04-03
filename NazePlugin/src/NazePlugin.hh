#ifndef NAZEPLUGIN_HH_
#define NAZEPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
//#include "../include/MotorControl.h"
#include "/home/stsc/work/ros_ws/sitl_ipc/include/sitl_ipc_sim.h"



#define MAX_MOTORS 4

//struct sitl_motor_t;

namespace gazebo
{
class NazePluginPrivate;

class GAZEBO_VISIBLE NazePlugin : public ModelPlugin
{
public:
  NazePlugin();

  ~NazePlugin();

  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  void HandleMotorCommand(struct sitl_motor_t* msg);
  void HandleResetWorld();
  
private:
  std::unique_ptr<NazePluginPrivate> data_;

  /// \brief transform from model orientation to x-forward and z-up
  ignition::math::Pose3d modelXYZToAirplaneXForwardZDown_;

  /// \brief transform from world frame to NED frame
  ignition::math::Pose3d gazeboXYZToNED_;

  void TimerCallback(const ros::TimerEvent &event);

  void SendSonarState() const;

  sensors::SonarSensorPtr LoadSonar(std::string const &name);

  sensors::ImuSensorPtr LoadImu(std::string const &name);

  void SendGpsState() const;

  sensors::GpsSensorPtr LoadGps(std::string const &name);


  void OnWorldUpdate();

  /// \brief Send state to ArduPilot
  void SendImuState() const;

  /// \brief Init ardupilot socket
  //bool InitArduPilotSockets(sdf::ElementPtr _sdf) const;

  void ApplyMotorForces(const double dt);
  void ResetPIDs();
  void LoadOrientation(sdf::ElementPtr sdf);
  void LoadRotorControls(sdf::ElementPtr sdf);
};

/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  /// should rename to servo_command here and in ArduPilot SIM_Gazebo.cpp
  float motorSpeed[MAX_MOTORS] = {0.0f};
  uint32_t flags;
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduPilot
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];
  /*  NOT MERGED IN MASTER YET
  /// \brief Model latitude in WGS84 system
  double latitude = 0.0;

  /// \brief Model longitude in WGS84 system
  double longitude = 0.0;

  /// \brief Model altitude from GPS
  double altitude = 0.0;

  /// \brief Model estimated from airspeed sensor (e.g. Pitot) in m/s
  double airspeed = 0.0;

  /// \brief Battery voltage. Default to -1 to use sitl estimator.
  double battery_voltage = -1.0;

  /// \brief Battery Current.
  double battery_current = 0.0;

  /// \brief Model rangefinder value. Default to -1 to use sitl rangefinder.
  double rangefinder = -1.0;
*/
};

} // namespace gazebo

#endif
