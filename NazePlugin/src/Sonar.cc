#include "Sonar.hh"
#include "sdfHelper.hh"
#include "fcl_types.h"
#include "fcl_sim_proxy.h"
#include "fcl_fc_proxy.h"

using namespace naze;
using namespace gazebo;

void Sonar::SendState()
{
    fcl_sonar_t sonar = { (uint32_t)(sensor_->Range() * 100.0f) };
    fcl_send_to_fc(eSonar, &sonar);

//    printf("sonar %d\n", msg.distance);
    //    msg.min_range = data_->sonar_sensor_->RangeMin();
    //    msg.max_range = data_->sonar_sensor_->RangeMax();
}
