#ifndef NAZEPLUGIN_HH_
#define NAZEPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include "/home/stsc/work/ros_ws/sitl_ipc/include/sitl_ipc_sim.h"

#define MAX_MOTORS 4

namespace gazebo
{
class NazePluginPrivate;

class GAZEBO_VISIBLE NazePlugin : public ModelPlugin
{
public:
  NazePlugin();
  ~NazePlugin();
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  void HandleMotorCommand(struct sitl_motor_t *msg);
  void HandleResetWorld();

private:
  std::unique_ptr<NazePluginPrivate> data_;

  void TimerCallback(const ros::TimerEvent &event);
  void OnWorldUpdate();
  void ApplyMotorForces(const double dt);
  void ResetPIDs();
  void LoadRotorControls(sdf::ElementPtr sdf);
};

} // namespace gazebo

#endif
