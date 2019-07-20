#include "Imu.hh"
#include "sdfHelper.hh"
#include "fcl_types.h"
#include "fcl_sim_proxy.h"
#include "fcl_fc_proxy.h"

using namespace naze;
using namespace gazebo;

bool Imu::Load(physics::ModelPtr model, sdf::ElementPtr sdf, std::string const &name)
{
    bool ret = BaseSensor::Load(model, name);

    // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
    // x-forward, y-left, z-up
    // to the aerospace convention: x-forward, y-right, z-down
    modelXYZToAirplaneXForwardZDown_ =
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
    if (sdf->HasElement("modelXYZToAirplaneXForwardZDown"))
    {
        modelXYZToAirplaneXForwardZDown_ =
            sdf->Get<ignition::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
    }

    // gazeboXYZToNED: from gazebo model frame: x-forward, y-left, z-up
    // to the aerospace convention: x-forward, y-right, z-down
    gazeboXYZToNED_ = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
    if (sdf->HasElement("gazeboXYZToNED"))
    {
        gazeboXYZToNED_ = sdf->Get<ignition::math::Pose3d>("gazeboXYZToNED");
    }

    return ret;
}

void Imu::SendState()
{
    // asssumed that the imu orientation is:
    //   x forward
    //   y right
    //   z down
    gzmsg << "Imu::SendState\n";

    // get linear acceleration in body frame
    const ignition::math::Vector3d linearAccel =
        sensor_->LinearAcceleration();

    // get angular velocity in body frame
    const ignition::math::Vector3d angularVel =
        sensor_->AngularVelocity();

    const ignition::math::Pose3d gazeboXYZToModelXForwardZDown =
        modelXYZToAirplaneXForwardZDown_ +
        model_->WorldPose();

    // get transform from world NED to Model frame
    const ignition::math::Pose3d NEDToModelXForwardZUp =
        gazeboXYZToModelXForwardZDown - gazeboXYZToNED_;

    const ignition::math::Vector3d velGazeboWorldFrame = model_->GetLink()->WorldLinearVel();
    const ignition::math::Vector3d velNEDFrame = gazeboXYZToNED_.Rot().RotateVectorReverse(velGazeboWorldFrame);

    fcl_fcstate_t state;
    state.imu.orientation_quat_w = NEDToModelXForwardZUp.Rot().W();
    state.imu.orientation_quat_x = NEDToModelXForwardZUp.Rot().X();
    state.imu.orientation_quat_y = NEDToModelXForwardZUp.Rot().Y();
    state.imu.orientation_quat_z = NEDToModelXForwardZUp.Rot().Z();

    state.imu.angular_velocity_r = angularVel.X();
    state.imu.angular_velocity_p = angularVel.Y();
    state.imu.angular_velocity_y = angularVel.Z();

    state.imu.linear_acceleration_x = linearAccel.X();
    state.imu.linear_acceleration_y = linearAccel.Y();
    state.imu.linear_acceleration_z = linearAccel.Z();

    state.pos.x = NEDToModelXForwardZUp.Pos().X();
    state.pos.y = NEDToModelXForwardZUp.Pos().Y();
    state.pos.z = NEDToModelXForwardZUp.Pos().Z();

    state.sim_time = model_->GetWorld()->SimTime().Double();
    fcl_send_to_fc(eFcstate, &state);
}