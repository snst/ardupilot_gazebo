#ifndef _SONAR_HH_
#define _SONAR_HH_

#include <gazebo/sensors/sensors.hh>

using namespace gazebo;

namespace naze
{

class Sonar
{
  public:
    void SendState() const;
    bool Load(physics::ModelPtr model, std::string const &name);

  protected:
    sensors::SonarSensorPtr sensor_;
};

} // namespace naze
#endif