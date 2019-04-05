#include "Sonar.hh"
#include "sdfHelper.hh"
#include "sitl_ipc_sim.h"

using namespace naze;
using namespace gazebo;

void Sonar::SendState()
{
    struct sitl_sonar_t msg;
    msg.distance = sensor_->Range();
    //    msg.min_range = data_->sonar_sensor_->RangeMin();
    //    msg.max_range = data_->sonar_sensor_->RangeMax();
    sitl_set_sonar(&msg);
}