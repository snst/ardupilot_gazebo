#ifndef _IMU_HH_
#define _IMU_HH_

#include <gazebo/sensors/sensors.hh>
#include "BaseSensor.hh"

using namespace gazebo;

namespace naze
{

class Imu : public BaseSensor<sensors::ImuSensor>
{
public:
  void SendState();
  bool Load(physics::ModelPtr model, sdf::ElementPtr sdf, std::string const &name);

protected:
  /// \brief transform from world frame to NED frame
  ignition::math::Pose3d gazeboXYZToNED_;
  /// \brief transform from model orientation to x-forward and z-up
  ignition::math::Pose3d modelXYZToAirplaneXForwardZDown_;
};

} // namespace naze
#endif