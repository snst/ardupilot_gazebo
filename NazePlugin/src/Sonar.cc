#include "Sonar.hh"
#include "sdfHelper.hh"
#include "/home/stsc/work/ros_ws/sitl_ipc/include/sitl_ipc_sim.h"

using namespace naze;
using namespace gazebo;

bool Sonar::Load(physics::ModelPtr model, std::string const &name)
{
    std::vector<std::string> scopedName = getSensorScopedName(model, name);

    if (scopedName.size() > 0)
    {
        gzmsg << "scopedName[0/" << scopedName.size() << "]: " << scopedName[0] << "\n";
        sensor_ = std::dynamic_pointer_cast<sensors::SonarSensor>(sensors::SensorManager::Instance()->GetSensor(scopedName[0]));
    }

    bool ret = (nullptr != sensor_);
    gzmsg << "Found sonar: " << ret << "\n";
    return ret;
}


void Sonar::SendState() const
{
    struct sitl_sonar_t msg;
    msg.distance = sensor_->Range();
    //    msg.min_range = data_->sonar_sensor_->RangeMin();
    //    msg.max_range = data_->sonar_sensor_->RangeMax();
    sitl_set_sonar(&msg);
}