#ifndef _BASE_SENSOR_HH_
#define _BASE_SENSOR_HH_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "sdfHelper.hh"

template <typename T>
class BaseSensor
{
  protected:
    gazebo::common::Time last_time;
    std::shared_ptr<T> sensor_;
    physics::ModelPtr model_;
    std::string name_;
    uint32_t n_;

  public:
    BaseSensor()
        : n_(0)
    {
    }
    virtual ~BaseSensor()
    {
    }

    virtual void SendState() = 0;

    bool Load(gazebo::physics::ModelPtr model, std::string const &name)
    {
        model_ = model;
        std::vector<std::string> scopedName = getSensorScopedName(model, name);

        if (scopedName.size() > 0)
        {
            name_ = scopedName[0];
            gzmsg << "scopedName[0/" << scopedName.size() << "]: " << scopedName[0] << "\n";
            sensor_ = std::dynamic_pointer_cast<T>(gazebo::sensors::SensorManager::Instance()->GetSensor(scopedName[0]));
        }
        else
        {
            name_ = name;
        }

        bool ret = (nullptr != sensor_);
        gzmsg << "Found " << name_ << ": " << ret << "\n";
        return ret;
    }

    virtual void Update(gazebo::common::Time const &cur_time, double ms)
    {
        double delta = (cur_time - last_time).Double() * 1000.0f;
        if (delta >= ms)
        {
            n_++;
            //            gzmsg << "<#" << n_ << ", " << delta << ", " << name_ << "\n";
            SendState();
            last_time = cur_time;
        }
    }
};

#endif