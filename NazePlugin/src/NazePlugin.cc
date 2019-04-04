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
#include "Sonar.hh"
#include "Gps.hh"
#include "Imu.hh"

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

    naze::Imu imu_;
    naze::Sonar sonar_;
    naze::Gps gps_;
};

NazePlugin::NazePlugin()
    : data_(new NazePluginPrivate)
{
    data_->is_online_ = false;
    gzmsg << "starting NazePlugin..\n";
    int i = 0;
    while (i > 0)
    {
        sleep(1);
        i--;
    }
    //raise(SIGTRAP);
    //__builtin_trap();
    //raise(SIGABRT);
}

NazePlugin::~NazePlugin()
{
}

void NazePlugin::TimerCallback(const ros::TimerEvent &event)
{
    gzmsg << "TimerCallback called!\n";
    //SendSonarState();
}

void NazePlugin::LoadRotorControls(sdf::ElementPtr sdf)
{
    // per control channel
    sdf::ElementPtr controlSDF;
    if (sdf->HasElement("control"))
    {
        controlSDF = sdf->GetElement("control");
    }
    else if (sdf->HasElement("rotor")) // stsc remove
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
            control.channel = atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
        }
        else
        {
            control.channel = this->data_->controls_.size();
            gzwarn << "[" << this->data_->model_name_ << "] "
                   << "id/channel attribute not specified, use order parsed ["
                   << control.channel << "].\n";
        }

        control.type = "VELOCITY";

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
            control.multiplier = controlSDF->Get<double>("multiplier"); // stsc +
        }
        else
        {
            gzdbg << "[" << this->data_->model_name_ << "] "
                  << "<multiplier> not specified,"
                  << " Default 1.\n";
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

    LoadRotorControls(sdf);

    this->data_->sonar_.Load(model, "iris_demo::iris::base_link::sonar");
    this->data_->imu_.Load(model, "iris_demo::iris::iris/imu_link::imu_sensor");
    this->data_->gps_.Load(model, sdf, "iris_demo::iris::base_link::gps");

    this->data_->imu_.LoadOrientation(sdf);

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
                this->data_->imu_.SendState();
                this->data_->last_imu_update_time_ = cur_time;
                iimu++;
            }

            if (gps_delta_time >= 0.1)
            {
                this->data_->gps_.SendState();
                this->data_->last_gps_update_time_ = cur_time;
                igps++;
            }

            if (sonar_delta_time >= 0.02)
            {
                //SendSonarState();
                this->data_->sonar_.SendState();
                this->data_->last_sonar_update_time_ = cur_time;
                isonar++;
            }

            printfxy(0, 26, "#%i imu #%i, sonar #%i, gps #%i\n", i, iimu, isonar, igps);
            i++;
        }
    }
    else
    {
        printf("skip time\n");
    }

    this->data_->last_controller_update_time_ = cur_time;
}

void NazePlugin::ApplyMotorForces(const double _dt)
{
    // update velocity PID for controls and apply force to joint
    for (size_t i = 0; i < this->data_->controls_.size(); ++i)
    {
        const double velTarget = this->data_->controls_[i].cmd /
                                 this->data_->controls_[i].rotorVelocitySlowdownSim;
        const double vel = this->data_->controls_[i].joint->GetVelocity(0);
        const double error = vel - velTarget;
        const double force = this->data_->controls_[i].pid.Update(error, _dt);
        this->data_->controls_[i].joint->SetForce(0, force);
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

void NazePlugin::HandleMotorCommand(struct sitl_motor_t *msg)
{
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

void NazePlugin::ResetPIDs()
{
    // Reset velocity PID for controls
    for (size_t i = 0; i < this->data_->controls_.size(); ++i)
    {
        this->data_->controls_[i].cmd = 0;
        this->data_->controls_[i].pid.SetCmd(0.0);
        this->data_->controls_[i].pid.Reset();
    }
}
