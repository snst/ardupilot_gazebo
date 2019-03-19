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
#include "sensor_msgs/Range.h"
#include "sdfHelper.hh"
#include "RotorControl.hh"
#include "ArduPilotSocket.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(NazePlugin)

class gazebo::NazePluginPrivate
{
  public:
    event::ConnectionPtr update_connection_;

    physics::ModelPtr model_;

    std::string model_name_;

    std::vector<RotorControl> controls_; /// \brief array of propellers

    gazebo::common::Time last_controller_update_time_;

    std::mutex mutex_;

    bool is_online_;

    sensors::ImuSensorPtr imu_sensor_;

    sensors::GpsSensorPtr gps_sensor_;

    //    sensors::RaySensorPtr rangefinderSensor;

    sensors::SonarSensorPtr sonar_sensor_;

    pthread_t motor_thread_;

    ros::NodeHandlePtr nh_;
    ros::Subscriber motor_sub_;

    ros::Timer timer_;
    ros::Publisher sonar_pub_;

    ::ArduPilotSocket socket_out_;
};

NazePlugin *gn = NULL;

NazePlugin::NazePlugin()
    : data_(new NazePluginPrivate)
{
    data_->is_online_ = false;
    gzmsg << "starting NazePlugin..\n";
    gn = this;
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

    gzmsg << "model_name: " << this->data_->model_name_ << "\n";

    LoadOrientation(sdf);
    LoadRotorControls(sdf);

    //default::naze_demo::iris_demo::iris::base_link::sonar
    this->data_->sonar_sensor_ = LoadSonar("iris_demo::iris::base_link::sonar");

    this->data_->imu_sensor_ = LoadImu("iris_demo::iris::iris/imu_link::imu_sensor");

    this->data_->nh_ = boost::make_shared<ros::NodeHandle>();
    //    this->data_->sonar_pub_ = this->data_->nh_->advertise<sensor_msgs::Range>("naze_sonar", 1000);
    //    this->data_->timer_ = this->data_->nh_->createTimer(ros::Duration(1), &NazePlugin::TimerCallback, this);

    gzmsg << "subscribe motor_data\n";

    ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<naze::MotorControl>(
        "/motor_data", 1, boost::bind(&NazePlugin::ReceiveMotorCommand, this, _1), ros::VoidPtr(), this->data_->nh_->getCallbackQueue());


    jointStatesSo.transport_hints = ros::TransportHints().unreliable();

    this->data_->motor_sub_ = this->data_->nh_->subscribe(jointStatesSo);

    if (!InitArduPilotSockets(sdf))
    {
        return;
    }

    this->data_->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&NazePlugin::OnUpdate, this));

    gzmsg << "[" << this->data_->model_name_ << "] "
          << "Naze ready to fly. The force will be with you" << std::endl;
}

void NazePlugin::OnUpdate()
{
    std::lock_guard<std::mutex> lock(this->data_->mutex_);

    const gazebo::common::Time cur_time = this->data_->model_->GetWorld()->SimTime();

    // Update the control surfaces and publish the new state.
    if (cur_time > this->data_->last_controller_update_time_)
    {
        //this->ReceiveMotorCommand();
        if (this->data_->is_online_)
        {
            this->ApplyMotorForces((cur_time - this->data_->last_controller_update_time_).Double());
            this->SendState();
        }
    }
    else
    {
        printf("skip time\n");
    }

    this->data_->last_controller_update_time_ = cur_time;
}


bool NazePlugin::InitArduPilotSockets(sdf::ElementPtr _sdf) const
{
    std::string fdm_addr = "127.0.0.1";
    uint16_t fdm_port_out = 9003;

    if (!this->data_->socket_out_.Connect(fdm_addr.c_str(),
                                          fdm_port_out))
    {
        gzerr << "[" << this->data_->model_name_ << "] "
              << "failed to bind with " << fdm_addr
              << ":" << fdm_port_out << " aborting plugin.\n";
        return false;
    }

    return true;
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

/////////////////////////////////////////////////
void NazePlugin::ReceiveMotorCommand(const naze::MotorControl::ConstPtr &msg)
{
    //gzmsg << "ReceiveMotorCommand!\n";

    float pkt[] = {msg->m0, msg->m1, msg->m2, msg->m3};

    if (msg->flags > 0)
    {
        printf("RESET!!\n");
        this->ResetPIDs();
        // Send reset world message
        transport::NodePtr node = transport::NodePtr(new transport::Node());
        node->Init();

        // Send reset world message
        transport::PublisherPtr worldControlPub =
            node->Advertise<msgs::WorldControl>("~/world_control");

        // Copied from MainWindow::OnResetWorld
        msgs::WorldControl msg;
        msg.mutable_reset()->set_all(true);
        worldControlPub->Publish(msg);
    }

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

void NazePlugin::SendState() const
{
    fdmPacket pkt;

    pkt.timestamp = this->data_->model_->GetWorld()->SimTime().Double();

    // asssumed that the imu orientation is:
    //   x forward
    //   y right
    //   z down

    // get linear acceleration in body frame
    const ignition::math::Vector3d linearAccel =
        this->data_->imu_sensor_->LinearAcceleration();

    // copy to pkt
    pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
    pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
    pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
    // gzerr << "lin accel [" << linearAccel << "]\n";

    // get angular velocity in body frame
    const ignition::math::Vector3d angularVel =
        this->data_->imu_sensor_->AngularVelocity();

    // copy to pkt
    pkt.imuAngularVelocityRPY[0] = angularVel.X();
    pkt.imuAngularVelocityRPY[1] = angularVel.Y();
    pkt.imuAngularVelocityRPY[2] = angularVel.Z();

    // get inertial pose and velocity
    // position of the uav in world frame
    // this position is used to calcualte bearing and distance
    // from starting location, then use that to update gps position.
    // The algorithm looks something like below (from ardupilot helper
    // libraries):
    //   bearing = to_degrees(atan2(position.y, position.x));
    //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
    //   (self.latitude, self.longitude) = util.gps_newpos(
    //    self.home_latitude, self.home_longitude, bearing, distance)
    // where xyz is in the NED directions.
    // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
    // around.
    // orientation of the uav in world NED frame -
    // assuming the world NED frame has xyz mapped to NED,
    // imuLink is NED - z down

    // model world pose brings us to model,
    // which for example zephyr has -y-forward, x-left, z-up
    // adding modelXYZToAirplaneXForwardZDown rotates
    //   from: model XYZ
    //   to: airplane x-forward, y-left, z-down
    const ignition::math::Pose3d gazeboXYZToModelXForwardZDown =
        this->modelXYZToAirplaneXForwardZDown_ +
        this->data_->model_->WorldPose();

    // get transform from world NED to Model frame
    const ignition::math::Pose3d NEDToModelXForwardZUp =
        gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED_;

    // gzerr << "ned to model [" << NEDToModelXForwardZUp << "]\n";

    // N
    pkt.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

    // E
    pkt.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

    // D
    pkt.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

    // imuOrientationQuat is the rotation from world NED frame
    // to the uav frame.
    pkt.imuOrientationQuat[0] = NEDToModelXForwardZUp.Rot().W();
    pkt.imuOrientationQuat[1] = NEDToModelXForwardZUp.Rot().X();
    pkt.imuOrientationQuat[2] = NEDToModelXForwardZUp.Rot().Y();
    pkt.imuOrientationQuat[3] = NEDToModelXForwardZUp.Rot().Z();

    // gzdbg << "imu [" << gazeboXYZToModelXForwardZDown.rot.GetAsEuler()
    //       << "]\n";
    // gzdbg << "ned [" << this->gazeboXYZToNED.rot.GetAsEuler() << "]\n";
    // gzdbg << "rot [" << NEDToModelXForwardZUp.rot.GetAsEuler() << "]\n";

    // Get NED velocity in body frame *
    // or...
    // Get model velocity in NED frame
    const ignition::math::Vector3d velGazeboWorldFrame =
        this->data_->model_->GetLink()->WorldLinearVel();
    const ignition::math::Vector3d velNEDFrame =
        this->gazeboXYZToNED_.Rot().RotateVectorReverse(velGazeboWorldFrame);
    pkt.velocityXYZ[0] = velNEDFrame.X();
    pkt.velocityXYZ[1] = velNEDFrame.Y();
    pkt.velocityXYZ[2] = velNEDFrame.Z();
    /* NOT MERGED IN MASTER YET
  if (!this->data_->gpsSensor)
    {

    }
    else {
        pkt.longitude = this->data_->gpsSensor->Longitude().Degree();
        pkt.latitude = this->data_->gpsSensor->Latitude().Degree();
        pkt.altitude = this->data_->gpsSensor->Altitude();
    }

    // TODO : make generic enough to accept sonar/gpuray etc. too
    if (!this->data_->rangefinderSensor)
    {

    } else {
        // Rangefinder value can not be send as Inf to ardupilot
        const double range = this->data_->rangefinderSensor->Range(0);
        pkt.rangefinder = std::isinf(range) ? 0.0 : range;
    }

  // airspeed :     wind = Vector3(environment.wind.x, environment.wind.y, environment.wind.z)
   // pkt.airspeed = (pkt.velocity - wind).length()
*/
    this->data_->socket_out_.Send(&pkt, sizeof(pkt));
}

void NazePlugin::ResetPIDs()
{
    // Reset velocity PID for controls
    for (size_t i = 0; i < this->data_->controls_.size(); ++i)
    {
        this->data_->controls_[i].cmd = 0;
        // this->data_->controls[i].pid.Reset();
    }
}
