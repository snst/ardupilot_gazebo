#ifndef ROTOR_CONTROL_HH
#define ROTOR_CONTROL_HH

#include <gazebo/common/common.hh>


using namespace gazebo;


class RotorControl
{
  public:
    RotorControl();

    int channel = 0; /// \brief control id / channel
    double cmd = 0;  /// \brief Next command to be applied to the propeller
    common::PID pid; /// \brief Velocity PID for motor control
    /// \brief RotorControl type. Can be:
    /// VELOCITY control velocity of joint
    /// POSITION control position of joint
    /// EFFORT control effort of joint
    std::string type;
    bool useForce = true;            /// \brief use force controler
    std::string jointName;           /// \brief RotorControl propeller joint.
    physics::JointPtr joint;         /// \brief RotorControl propeller joint.
    double multiplier = 1;           /// \brief direction multiplier for this control
    double offset = 0;               /// \brief input command offset
    double rotorVelocitySlowdownSim; /// \brief unused coefficients
    double frequencyCutoff;
    double samplingRate;
    ignition::math::OnePole<double> filter;
    static double kDefaultRotorVelocitySlowdownSim;
    static double kDefaultFrequencyCutoff;
    static double kDefaultSamplingRate;
};

#endif