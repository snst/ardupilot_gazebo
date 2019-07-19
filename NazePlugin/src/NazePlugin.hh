#ifndef NAZEPLUGIN_HH_
#define NAZEPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "fcl_types.h"

//#define MAX_MOTORS 4

namespace gazebo
{
class NazePluginPrivate;

class GAZEBO_VISIBLE NazePlugin : public ModelPlugin
{
public:
  NazePlugin();
  ~NazePlugin();
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  void HandleMotorCommand(fcl_motor_t* motor);
  void HandleResetWorld();

private:
  std::unique_ptr<NazePluginPrivate> data_;

  //void TimerCallback(const ros::TimerEvent &event);
  void OnWorldUpdate();
  void ApplyMotorForces(const double dt);
  void ResetPIDs();
  void LoadRotorControls(sdf::ElementPtr sdf);
};

} // namespace gazebo

#endif
