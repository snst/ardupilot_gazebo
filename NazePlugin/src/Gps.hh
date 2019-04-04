#ifndef _GPS_HH_
#define _GPS_HH_

#include <gazebo/sensors/sensors.hh>

using namespace gazebo;

namespace naze
{

class Gps
{
public:
  void SendState() const;
  bool Load(physics::ModelPtr model, sdf::ElementPtr sdf, std::string const &name);

protected:
  sensors::GpsSensorPtr sensor_;
  double reference_latitude_;
  double reference_longitude_;
  double reference_altitude_;
};

} // namespace naze
#endif