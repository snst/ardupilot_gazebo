#include "Gps.hh"
#include "sdfHelper.hh"
#include "sitl_ipc_sim.h"

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
    struct sitl_gps_t data;
    data.latitude = reference_latitude_ - sensor_->Latitude().Degree();
    data.longitude = reference_longitude_ - sensor_->Longitude().Degree();
    data.altitude = reference_altitude_ + sensor_->Altitude();
    data.status = 0; //sensor_msgs::NavSatStatus::STATUS_FIX;
    sitl_set_gps(&data);
}