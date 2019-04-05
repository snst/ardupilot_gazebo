#ifndef _GPS_HH_
#define _GPS_HH_

#include <gazebo/sensors/sensors.hh>
#include "BaseSensor.hh"

using namespace gazebo;

namespace naze
{

class Gps : public BaseSensor<sensors::GpsSensor>
{
public:
  Gps();
  void SendState();
  bool Load(physics::ModelPtr model, sdf::ElementPtr sdf, std::string const &name);

protected:
  double reference_latitude_;
  double reference_longitude_;
  double reference_altitude_;
};

} // namespace naze
#endif