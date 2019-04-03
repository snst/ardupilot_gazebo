#ifndef _IMU_HH_
#define _IMU_HH_

#include <gazebo/sensors/sensors.hh>

using namespace gazebo;

namespace naze
{

class Imu
{
  public:
    void SendState() const;
    bool Load(physics::ModelPtr model, std::string const &name);
    void LoadOrientation(sdf::ElementPtr sdf);

  protected:
    /// \brief transform from world frame to NED frame
    ignition::math::Pose3d gazeboXYZToNED_;
    /// \brief transform from model orientation to x-forward and z-up
    ignition::math::Pose3d modelXYZToAirplaneXForwardZDown_;

    sensors::ImuSensorPtr sensor_;
    physics::ModelPtr model_;
};

} // namespace naze
#endif