#include <mutex>
#include <string>
#include <vector>
#include <sdf/sdf.hh>
#include <ignition/math/Filter.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "include/NazePlugin.hh"
#include "sensor_msgs/Range.h"
#include "sdfHelper.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(NazePlugin)

class gazebo::NazePluginPrivate
{
  public:
    event::ConnectionPtr update_connection_;

    physics::ModelPtr model_;

    std::string model_name_;

    //std::vector<Control> controls_;

    gazebo::common::Time last_controller_update_time_;

    std::mutex mutex_;

    sensors::ImuSensorPtr imu_sensor_;

    sensors::GpsSensorPtr gps_sensor_;

    //    sensors::RaySensorPtr rangefinderSensor;

    sensors::SonarSensorPtr sonar_sensor_;

    pthread_t motor_thread_;

    ros::NodeHandlePtr nh_;
    ros::Timer timer_;
    ros::Publisher sonar_pub_;
};

NazePlugin::NazePlugin()
    : data_(new NazePluginPrivate)
{
}

NazePlugin::~NazePlugin()
{
}

void NazePlugin::SendSonarState()
{
    gzmsg << "SendSonarState:" << data_->sonar_sensor_->Range() << "\n";
    sensor_msgs::Range msg;
    msg.min_range = data_->sonar_sensor_->RangeMin();
    msg.max_range = data_->sonar_sensor_->RangeMax();
    msg.range = data_->sonar_sensor_->Range();
    ROS_INFO("%f", msg.range);
    data_->sonar_pub_.publish(msg);
}

void NazePlugin::TimerCallback(const ros::TimerEvent &event)
{
    gzmsg << "TimerCallback called!\n";
    SendSonarState();
}

bool NazePlugin::LoadSonar(std::string const &name)
{
    std::vector<std::string> sonar_scoped_name = getSensorScopedName(this->data_->model_, name);

    if (sonar_scoped_name.size() > 0)
    {
        gzmsg << "sonar_scoped_name[0/" << sonar_scoped_name.size() << "]: " << sonar_scoped_name[0] << "\n";
        this->data_->sonar_sensor_ = std::dynamic_pointer_cast<sensors::SonarSensor>(sensors::SensorManager::Instance()->GetSensor(sonar_scoped_name[0]));
    }

    bool ret = (nullptr != this->data_->sonar_sensor_);
    gzmsg << "Found sonar: " << ret << "\n";
    return ret;
}

bool NazePlugin::LoadImu(std::string const &name)
{
    std::vector<std::string> imu_scoped_name = getSensorScopedName(this->data_->model_, name);

    if (imu_scoped_name.size() > 0)
    {
        gzmsg << "imu_scoped_name[0/" << imu_scoped_name.size() << "]: " << imu_scoped_name[0] << "\n";
        this->data_->imu_sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor(imu_scoped_name[0]));
    }

    bool ret = (nullptr != this->data_->imu_sensor_);
    gzmsg << "Found imu: " << ret << "\n";
    return ret;
}

void NazePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    this->data_->model_ = model;
    this->data_->model_name_ = model->GetName();

    gzmsg << "model_name: " << this->data_->model_name_ << "\n";

    this->data_->nh_ = boost::make_shared<ros::NodeHandle>();
    this->data_->sonar_pub_ = this->data_->nh_->advertise<sensor_msgs::Range>("naze_sonar", 1000);

    //default::naze_demo::iris_demo::iris::base_link::sonar
    LoadSonar("iris_demo::iris::base_link::sonar");

    LoadImu("iris_demo::iris::iris/imu_link::imu_sensor");

    this->data_->timer_ = this->data_->nh_->createTimer(ros::Duration(1), &NazePlugin::TimerCallback, this);

    this->data_->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&NazePlugin::OnUpdate, this));

    gzmsg << "[" << this->data_->model_name_ << "] "
          << "Naze ready to fly. The force will be with you" << std::endl;
}

void NazePlugin::OnUpdate()
{
    std::lock_guard<std::mutex> lock(this->data_->mutex_);

    const gazebo::common::Time cur_time =
        this->data_->model_->GetWorld()->SimTime();
}

void *NazePlugin::MotorWorker(void *ptr)
{
    printf("MotorWorker started\n");
    NazePlugin *plugin = (NazePlugin *)ptr;
}
