#include "Gps.hh"
#include "sdfHelper.hh"
#include "/home/stsc/work/ros_ws/sitl_ipc/include/sitl_ipc_sim.h"

using namespace naze;
using namespace gazebo;

bool Gps::Load(physics::ModelPtr model, std::string const &name)
{
    std::vector<std::string> scopedName = getSensorScopedName(model, name);

    if (scopedName.size() > 0)
    {
        gzmsg << "scopedName[0/" << scopedName.size() << "]: " << scopedName[0] << "\n";
        sensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(sensors::SensorManager::Instance()->GetSensor(scopedName[0]));
    }

    bool ret = (nullptr != sensor_);
    gzmsg << "Found gps: " << ret << "\n";
    return ret;
}


static const double DEFAULT_REFERENCE_LATITUDE = 49.48696;
static const double DEFAULT_REFERENCE_LONGITUDE = 11.1249838;
static const double DEFAULT_REFERENCE_HEADING = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE = 0.0;


void Gps::SendState() const
{
    double reference_latitude_ = DEFAULT_REFERENCE_LATITUDE;
    double reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
    double reference_altitude_ = DEFAULT_REFERENCE_ALTITUDE;

    struct sitl_gps_t data;
    data.latitude = reference_latitude_ - sensor_->Latitude().Degree();
    data.longitude = reference_longitude_ - sensor_->Longitude().Degree();
    data.altitude = reference_altitude_ + sensor_->Altitude();
    data.status = 0; //sensor_msgs::NavSatStatus::STATUS_FIX;
    sitl_set_gps(&data);
}