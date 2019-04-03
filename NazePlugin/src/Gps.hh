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
    bool Load(physics::ModelPtr model, std::string const &name);

  protected:
    sensors::GpsSensorPtr sensor_;
};

} // namespace naze
#endif