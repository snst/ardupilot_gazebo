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
#include "NazePlugin.hh"
#include "sdfHelper.hh"
#include "RotorControl.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(NazePlugin)

void printfxy(int x, int y, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    printf("\033[%d;%dH", 15 + y, x);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
}

class gazebo::NazePluginPrivate
{
  public:
    event::ConnectionPtr update_connection_;
    physics::ModelPtr model_;
    std::string model_name_;
    std::vector<RotorControl> controls_; /// \brief array of propellers
    gazebo::common::Time last_controller_update_time_;
    gazebo::common::Time last_imu_update_time_;
    gazebo::common::Time last_sonar_update_time_;
    gazebo::common::Time last_gps_update_time_;

    std::mutex mutex_;
    bool is_online_;
    sensors::ImuSensorPtr imu_sensor_;
    sensors::GpsSensorPtr gps_sensor_;
    sensors::SonarSensorPtr sonar_sensor_;
};

NazePlugin::NazePlugin()
    : data_(new NazePluginPrivate)
{
    data_->is_online_ = false;
    gzmsg << "starting NazePlugin..\n";
}

NazePlugin::~NazePlugin()
{
}

void NazePlugin::SendSonarState() const
{
    struct sitl_sonar_t msg;
    msg.distance = data_->sonar_sensor_->Range();
    //    msg.min_range = data_->sonar_sensor_->RangeMin();
    //    msg.max_range = data_->sonar_sensor_->RangeMax();
    sitl_set_sonar(&msg);
}

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0 / 298.257223563;
static const double excentrity2 = 2 * flattening - flattening * flattening;

// 49.482097,11.0880847
// 49.482098,11.0885358
// default reference position
//49.48696,11.1249838
static const double DEFAULT_REFERENCE_LATITUDE = 49.48696;
static const double DEFAULT_REFERENCE_LONGITUDE = 11.1249838;
static const double DEFAULT_REFERENCE_HEADING = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE = 0.0;

void NazePlugin::SendGpsState() const
{
    double reference_latitude_ = DEFAULT_REFERENCE_LATITUDE;
    double reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
    double reference_altitude_ = DEFAULT_REFERENCE_ALTITUDE;

    struct sitl_gps_t data;
    data.latitude = reference_latitude_ - data_->gps_sensor_->Latitude().Degree();
    data.longitude = reference_longitude_ - data_->gps_sensor_->Longitude().Degree();
    data.altitude = reference_altitude_ + data_->gps_sensor_->Altitude();
    data.status = 0; //sensor_msgs::NavSatStatus::STATUS_FIX;
    sitl_set_gps(&data);

    //    printf("gps alt=%f, lon=%f, lat=%f // lon=%f, lat=%f \n", alt, lon, lat, fix.latitude, fix.longitude);
}

sensors::GpsSensorPtr NazePlugin::LoadGps(std::string const &name)
{
    sensors::GpsSensorPtr sensor;
    std::vector<std::string> scoped_name = getSensorScopedName(this->data_->model_, name);

    if (scoped_name.size() > 0)
    {
        gzmsg << "gps scoped_name[0/" << scoped_name.size() << "]: " << scoped_name[0] << "\n";
        sensor = std::dynamic_pointer_cast<sensors::GpsSensor>(sensors::SensorManager::Instance()->GetSensor(scoped_name[0]));
    }

    bool ret = (nullptr != sensor);
    gzmsg << "Found gps: " << ret << "\n";
    return sensor;
}

void NazePlugin::TimerCallback(const ros::TimerEvent &event)
{
    gzmsg << "TimerCallback called!\n";
    SendSonarState();
}

sensors::SonarSensorPtr NazePlugin::LoadSonar(std::string const &name)
{
    sensors::SonarSensorPtr sonar;
    std::vector<std::string> sonar_scoped_name = getSensorScopedName(this->data_->model_, name);

    if (sonar_scoped_name.size() > 0)
    {
        gzmsg << "sonar_scoped_name[0/" << sonar_scoped_name.size() << "]: " << sonar_scoped_name[0] << "\n";
        sonar = std::dynamic_pointer_cast<sensors::SonarSensor>(sensors::SensorManager::Instance()->GetSensor(sonar_scoped_name[0]));
    }

    bool ret = (nullptr != sonar);
    gzmsg << "Found sonar: " << ret << "\n";
    return sonar;
}

sensors::ImuSensorPtr NazePlugin::LoadImu(std::string const &name)
{
    sensors::ImuSensorPtr imu;
    std::vector<std::string> imu_scoped_name = getSensorScopedName(this->data_->model_, name);

    if (imu_scoped_name.size() > 0)
    {
        gzmsg << "imu_scoped_name[0/" << imu_scoped_name.size() << "]: " << imu_scoped_name[0] << "\n";
        imu = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor(imu_scoped_name[0]));
    }

    bool ret = (nullptr != imu);
    gzmsg << "Found imu: " << ret << "\n";
    return imu;
}

void NazePlugin::LoadOrientation(sdf::ElementPtr sdf)
{
    // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
    // x-forward, y-left, z-up
    // to the aerospace convention: x-forward, y-right, z-down
    this->modelXYZToAirplaneXForwardZDown_ =
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
    if (sdf->HasElement("modelXYZToAirplaneXForwardZDown"))
    {
        this->modelXYZToAirplaneXForwardZDown_ =
            sdf->Get<ignition::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
    }

    // gazeboXYZToNED: from gazebo model frame: x-forward, y-left, z-up
    // to the aerospace convention: x-forward, y-right, z-down
    this->gazeboXYZToNED_ = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
    if (sdf->HasElement("gazeboXYZToNED"))
    {
        this->gazeboXYZToNED_ = sdf->Get<ignition::math::Pose3d>("gazeboXYZToNED");
    }
}

void NazePlugin::LoadRotorControls(sdf::ElementPtr sdf)
{
    // per control channel
    sdf::ElementPtr controlSDF;
    if (sdf->HasElement("control"))
    {
        controlSDF = sdf->GetElement("control");
    }
    else if (sdf->HasElement("rotor"))
    {
        gzwarn << "[" << this->data_->model_name_ << "] "
               << "please deprecate <rotor> block, use <control> block instead.\n";
        controlSDF = sdf->GetElement("rotor");
    }

    while (controlSDF)
    {
        RotorControl control;

        if (controlSDF->HasAttribute("channel"))
        {
            control.channel =
                atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
        }
        else if (controlSDF->HasAttribute("id"))
        {
            gzwarn << "[" << this->data_->model_name_ << "] "
                   << "please deprecate attribute id, use channel instead.\n";
            control.channel =
                atoi(controlSDF->GetAttribute("id")->GetAsString().c_str());
        }
        else
        {
            control.channel = this->data_->controls_.size();
            gzwarn << "[" << this->data_->model_name_ << "] "
                   << "id/channel attribute not specified, use order parsed ["
                   << control.channel << "].\n";
        }

        if (controlSDF->HasElement("type"))
        {
            control.type = controlSDF->Get<std::string>("type");
        }
        else
        {
            gzerr << "[" << this->data_->model_name_ << "] "
                  << "Control type not specified,"
                  << " using velocity control by default.\n";
            control.type = "VELOCITY";
        }

        if (control.type != "VELOCITY" &&
            control.type != "POSITION" &&
            control.type != "EFFORT")
        {
            gzwarn << "[" << this->data_->model_name_ << "] "
                   << "Control type [" << control.type
                   << "] not recognized, must be one of VELOCITY, POSITION, EFFORT."
                   << " default to VELOCITY.\n";
            control.type = "VELOCITY";
        }

        if (controlSDF->HasElement("useForce"))
        {
            control.useForce = controlSDF->Get<bool>("useForce");
        }

        if (controlSDF->HasElement("jointName"))
        {
            control.jointName = controlSDF->Get<std::string>("jointName");
        }
        else
        {
            gzerr << "[" << this->data_->model_name_ << "] "
                  << "Please specify a jointName,"
                  << " where the control channel is attached.\n";
        }

        // Get the pointer to the joint.
        control.joint = this->data_->model_->GetJoint(control.jointName);
        if (control.joint == nullptr)
        {
            gzerr << "[" << this->data_->model_name_ << "] "
                  << "Couldn't find specified joint ["
                  << control.jointName << "]. This plugin will not run.\n";
            return;
        }

        if (controlSDF->HasElement("multiplier"))
        {
            // overwrite turningDirection, deprecated.
            control.multiplier = controlSDF->Get<double>("multiplier");
        }
        else if (controlSDF->HasElement("turningDirection"))
        {
            gzwarn << "[" << this->data_->model_name_ << "] "
                   << "<turningDirection> is deprecated. Please use"
                   << " <multiplier>. Map 'cw' to '-1' and 'ccw' to '1'.\n";
            std::string turningDirection = controlSDF->Get<std::string>(
                "turningDirection");
            // special cases mimic from controls_gazebo_plugins
            if (turningDirection == "cw")
            {
                control.multiplier = -1;
            }
            else if (turningDirection == "ccw")
            {
                control.multiplier = 1;
            }
            else
            {
                gzdbg << "[" << this->data_->model_name_ << "] "
                      << "not string, check turningDirection as float\n";
                control.multiplier = controlSDF->Get<double>("turningDirection");
            }
        }
        else
        {
            gzdbg << "[" << this->data_->model_name_ << "] "
                  << "<multiplier> (or deprecated <turningDirection>) not specified,"
                  << " Default 1 (or deprecated <turningDirection> 'ccw').\n";
            control.multiplier = 1;
        }

        if (controlSDF->HasElement("offset"))
        {
            control.offset = controlSDF->Get<double>("offset");
        }
        else
        {
            gzdbg << "[" << this->data_->model_name_ << "] "
                  << "<offset> not specified, default to 0.\n";
            control.offset = 0;
        }

        getSdfParam<double>(controlSDF, "rotorVelocitySlowdownSim",
                            control.rotorVelocitySlowdownSim, 1);

        if (ignition::math::equal(control.rotorVelocitySlowdownSim, 0.0))
        {
            gzwarn << "[" << this->data_->model_name_ << "] "
                   << "control for joint [" << control.jointName
                   << "] rotorVelocitySlowdownSim is zero,"
                   << " assume no slowdown.\n";
            control.rotorVelocitySlowdownSim = 1.0;
        }

        getSdfParam<double>(controlSDF, "frequencyCutoff",
                            control.frequencyCutoff, control.frequencyCutoff);
        getSdfParam<double>(controlSDF, "samplingRate",
                            control.samplingRate, control.samplingRate);

        // use gazebo::math::Filter
        control.filter.Fc(control.frequencyCutoff, control.samplingRate);

        // initialize filter to zero value
        control.filter.Set(0.0);

        // note to use this filter, do
        // stateFiltered = filter.Process(stateRaw);

        // Overload the PID parameters if they are available.
        double param;
        // carry over from ArduCopter plugin
        getSdfParam<double>(controlSDF, "vel_p_gain", param,
                            control.pid.GetPGain());
        control.pid.SetPGain(param);

        getSdfParam<double>(controlSDF, "vel_i_gain", param,
                            control.pid.GetIGain());
        control.pid.SetIGain(param);

        getSdfParam<double>(controlSDF, "vel_d_gain", param,
                            control.pid.GetDGain());
        control.pid.SetDGain(param);

        getSdfParam<double>(controlSDF, "vel_i_max", param,
                            control.pid.GetIMax());
        control.pid.SetIMax(param);

        getSdfParam<double>(controlSDF, "vel_i_min", param,
                            control.pid.GetIMin());
        control.pid.SetIMin(param);

        getSdfParam<double>(controlSDF, "vel_cmd_max", param,
                            control.pid.GetCmdMax());
        control.pid.SetCmdMax(param);

        getSdfParam<double>(controlSDF, "vel_cmd_min", param,
                            control.pid.GetCmdMin());
        control.pid.SetCmdMin(param);

        // new params, overwrite old params if exist
        getSdfParam<double>(controlSDF, "p_gain", param,
                            control.pid.GetPGain());
        control.pid.SetPGain(param);

        getSdfParam<double>(controlSDF, "i_gain", param,
                            control.pid.GetIGain());
        control.pid.SetIGain(param);

        getSdfParam<double>(controlSDF, "d_gain", param,
                            control.pid.GetDGain());
        control.pid.SetDGain(param);

        getSdfParam<double>(controlSDF, "i_max", param,
                            control.pid.GetIMax());
        control.pid.SetIMax(param);

        getSdfParam<double>(controlSDF, "i_min", param,
                            control.pid.GetIMin());
        control.pid.SetIMin(param);

        getSdfParam<double>(controlSDF, "cmd_max", param,
                            control.pid.GetCmdMax());
        control.pid.SetCmdMax(param);

        getSdfParam<double>(controlSDF, "cmd_min", param,
                            control.pid.GetCmdMin());
        control.pid.SetCmdMin(param);

        // set pid initial command
        control.pid.SetCmd(0.0);

        this->data_->controls_.push_back(control);
        controlSDF = controlSDF->GetNextElement("control");
    }
}

void NazePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    this->data_->model_ = model;
    this->data_->model_name_ = model->GetName();

    gzmsg << "model_name: " << this->data_->model_name_ << std::endl;

    LoadOrientation(sdf);
    LoadRotorControls(sdf);

    this->data_->sonar_sensor_ = LoadSonar("iris_demo::iris::base_link::sonar");
    this->data_->imu_sensor_ = LoadImu("iris_demo::iris::iris/imu_link::imu_sensor");
    this->data_->gps_sensor_ = LoadGps("iris_demo::iris::base_link::gps");

    sitl_start_ipc();

    sitl_register_motor_callback(boost::bind(&NazePlugin::HandleMotorCommand, this, _1));
    sitl_register_reset_world_callback(boost::bind(&NazePlugin::HandleResetWorld, this));

    this->data_->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&NazePlugin::OnWorldUpdate, this));

    gzmsg << "[" << this->data_->model_name_ << "] "
          << "Naze ready to fly. The force will be with you" << std::endl;
}

void NazePlugin::OnWorldUpdate()
{
    std::lock_guard<std::mutex> lock(this->data_->mutex_);
    static int i = 0;
    const gazebo::common::Time cur_time = this->data_->model_->GetWorld()->SimTime();
    double delta_time = (cur_time - this->data_->last_controller_update_time_).Double();
    double imu_delta_time = (cur_time - this->data_->last_imu_update_time_).Double();
    double gps_delta_time = (cur_time - this->data_->last_gps_update_time_).Double();
    double sonar_delta_time = (cur_time - this->data_->last_sonar_update_time_).Double();

    static int iimu = 0, igps = 0, isonar = 0;
    // Update the control surfaces and publish the new state.
    if (delta_time > 0.0)
    {
        printfxy(0, 25, "#%i is_online %i, delta %f\n", i, this->data_->is_online_, delta_time);
        if (this->data_->is_online_)
        {
            ApplyMotorForces(delta_time);

            if (imu_delta_time >= 0.0)
            {
                SendImuState();
                this->data_->last_imu_update_time_ = cur_time;
                iimu++;
            }

            if (gps_delta_time >= 0.1)
            {
                SendGpsState();
                this->data_->last_gps_update_time_ = cur_time;
                igps++;
            }

            if (sonar_delta_time >= 0.02)
            {
                SendSonarState();
                this->data_->last_sonar_update_time_ = cur_time;
                isonar++;
            }

            printfxy(0, 26, "#%i imu #%i, sonar #%i, gps #%i\n", i, iimu, isonar, igps);
            i++;

            //();
            //();
        }
    }
    else
    {
        printf("skip time\n");
    }

    this->data_->last_controller_update_time_ = cur_time;
}

/////////////////////////////////////////////////
void NazePlugin::ApplyMotorForces(const double _dt)
{
    // update velocity PID for controls and apply force to joint
    for (size_t i = 0; i < this->data_->controls_.size(); ++i)
    {
        if (this->data_->controls_[i].useForce)
        {
            if (this->data_->controls_[i].type == "VELOCITY")
            {
                const double velTarget = this->data_->controls_[i].cmd /
                                         this->data_->controls_[i].rotorVelocitySlowdownSim;
                const double vel = this->data_->controls_[i].joint->GetVelocity(0);
                const double error = vel - velTarget;
                const double force = this->data_->controls_[i].pid.Update(error, _dt);
                this->data_->controls_[i].joint->SetForce(0, force);
            }
            else if (this->data_->controls_[i].type == "POSITION")
            {
                const double posTarget = this->data_->controls_[i].cmd;
                const double pos = this->data_->controls_[i].joint->Position();
                const double error = pos - posTarget;
                const double force = this->data_->controls_[i].pid.Update(error, _dt);
                this->data_->controls_[i].joint->SetForce(0, force);
            }
            else if (this->data_->controls_[i].type == "EFFORT")
            {
                const double force = this->data_->controls_[i].cmd;
                this->data_->controls_[i].joint->SetForce(0, force);
            }
            else
            {
                // do nothing
            }
        }
        else
        {
            if (this->data_->controls_[i].type == "VELOCITY")
            {
                this->data_->controls_[i].joint->SetVelocity(0, this->data_->controls_[i].cmd);
            }
            else if (this->data_->controls_[i].type == "POSITION")
            {
                this->data_->controls_[i].joint->SetPosition(0, this->data_->controls_[i].cmd);
            }
            else if (this->data_->controls_[i].type == "EFFORT")
            {
                const double force = this->data_->controls_[i].cmd;
                this->data_->controls_[i].joint->SetForce(0, force);
            }
            else
            {
                // do nothing
            }
        }
    }
}

void NazePlugin::HandleResetWorld()
{
    printf("RESET!!\n");
    this->ResetPIDs();
    // Send reset world message
    transport::NodePtr node = transport::NodePtr(new transport::Node());
    node->Init();

    // Send reset world message
    transport::PublisherPtr worldControlPub =
        node->Advertise<msgs::WorldControl>("~/world_control");

    // Copied from MainWindow::OnHandleResetWorld
    msgs::WorldControl msg;
    msg.mutable_reset()->set_all(true);
    worldControlPub->Publish(msg);

    this->data_->last_controller_update_time_ = 0;
    this->data_->last_sonar_update_time_ = 0;
    this->data_->last_imu_update_time_ = 0;
    this->data_->last_gps_update_time_ = 0;
}

/////////////////////////////////////////////////
void NazePlugin::HandleMotorCommand(struct sitl_motor_t *msg)
{
    //gzmsg << "HandleMotorCommand!\n";

    float pkt[] = {msg->motor[0], msg->motor[1], msg->motor[2], msg->motor[3]};

    const ssize_t recvChannels = 4;

    this->data_->is_online_ = true;

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->data_->controls_.size(); ++i)
    {
        if (i < MAX_MOTORS)
        {
            if (this->data_->controls_[i].channel < recvChannels)
            {
                // bound incoming cmd between -1 and 1
                const double cmd = ignition::math::clamp(
                    pkt[this->data_->controls_[i].channel],
                    -1.0f, 1.0f);
                this->data_->controls_[i].cmd =
                    this->data_->controls_[i].multiplier * (cmd + this->data_->controls_[i].offset);
            }
        }
    }
}

void NazePlugin::SendImuState() const
{
    // asssumed that the imu orientation is:
    //   x forward
    //   y right
    //   z down

    // get linear acceleration in body frame
    const ignition::math::Vector3d linearAccel =
        this->data_->imu_sensor_->LinearAcceleration();

    // get angular velocity in body frame
    const ignition::math::Vector3d angularVel =
        this->data_->imu_sensor_->AngularVelocity();

    const ignition::math::Pose3d gazeboXYZToModelXForwardZDown =
        this->modelXYZToAirplaneXForwardZDown_ +
        this->data_->model_->WorldPose();

    // get transform from world NED to Model frame
    const ignition::math::Pose3d NEDToModelXForwardZUp =
        gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED_;

    const ignition::math::Vector3d velGazeboWorldFrame = this->data_->model_->GetLink()->WorldLinearVel();
    const ignition::math::Vector3d velNEDFrame = this->gazeboXYZToNED_.Rot().RotateVectorReverse(velGazeboWorldFrame);

    struct sitl_imu_t imu;
    imu.orientation_quat_w = NEDToModelXForwardZUp.Rot().W();
    imu.orientation_quat_x = NEDToModelXForwardZUp.Rot().X();
    imu.orientation_quat_y = NEDToModelXForwardZUp.Rot().Y();
    imu.orientation_quat_z = NEDToModelXForwardZUp.Rot().Z();

    imu.angular_velocity_r = angularVel.X();
    imu.angular_velocity_p = angularVel.Y();
    imu.angular_velocity_y = angularVel.Z();

    imu.linear_acceleration_x = linearAccel.X();
    imu.linear_acceleration_y = linearAccel.Y();
    imu.linear_acceleration_z = linearAccel.Z();
    sitl_set_imu(&imu);

    struct sitl_pos_t pos;
    pos.x = NEDToModelXForwardZUp.Pos().X();
    pos.y = NEDToModelXForwardZUp.Pos().Y();
    pos.z = NEDToModelXForwardZUp.Pos().Z();
    sitl_set_pos(&pos);

    double simtime = this->data_->model_->GetWorld()->SimTime().Double();
    sitl_set_simtime(simtime);
}

void NazePlugin::ResetPIDs()
{
    // Reset velocity PID for controls
    for (size_t i = 0; i < this->data_->controls_.size(); ++i)
    {
        this->data_->controls_[i].cmd = 0;
        this->data_->controls_[i].pid.Reset();
    }
}
