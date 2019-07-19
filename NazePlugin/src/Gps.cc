#include "Gps.hh"
#include "sdfHelper.hh"
#include "fcl_types.h"
#include "fcl_sim_proxy.h"
#include "fcl_fc_proxy.h"

using namespace naze;
using namespace gazebo;

Gps::Gps()
    : reference_latitude_(0.0f), reference_longitude_(0.0f), reference_altitude_(0.0f)
{
}

bool Gps::Load(physics::ModelPtr model, sdf::ElementPtr sdf, std::string const &name)
{
    bool ret = BaseSensor::Load(model, name);

    if (sdf->HasElement("gps_latitude"))
    {
        reference_latitude_ = sdf->Get<double>("gps_latitude");
    }
    if (sdf->HasElement("gps_longitude"))
    {
        reference_longitude_ = sdf->Get<double>("gps_longitude");
    }
    if (sdf->HasElement("gps_altitude"))
    {
        reference_altitude_ = sdf->Get<double>("gps_altitude");
    }

    gzmsg << "gps_latitude: " << reference_latitude_ << ", gps_longitude: " << reference_longitude_ << ", gps_altitude: " << reference_altitude_ << "\n";
    return ret;
}

void Gps::SendState()
{
    fcl_gps_t gps;
    gps.latitude = reference_latitude_ - sensor_->Latitude().Degree();
    gps.longitude = reference_longitude_ - sensor_->Longitude().Degree();
    gps.altitude = reference_altitude_ + sensor_->Altitude();
    gps.status = 0; //sensor_msgs::NavSatStatus::STATUS_FIX;
    gps.satellites = 8;
    fcl_send_to_fc(eGps, &gps);
}