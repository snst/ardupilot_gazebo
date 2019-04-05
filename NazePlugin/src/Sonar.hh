#ifndef _SONAR_HH_
#define _SONAR_HH_

#include <gazebo/sensors/sensors.hh>
#include "BaseSensor.hh"

using namespace gazebo;

namespace naze
{

class Sonar : public BaseSensor<sensors::SonarSensor>
{
public:
  void SendState();
};

} // namespace naze
#endif