#ifndef NAZEPLUGIN_HH_
#define NAZEPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo
{
class NazePluginPrivate;

class GAZEBO_VISIBLE NazePlugin : public ModelPlugin
{
  public:
    NazePlugin();

    ~NazePlugin();

    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    static void *MotorWorker(void *ptr);

  private:
    std::unique_ptr<NazePluginPrivate> data_;

    void TimerCallback(const ros::TimerEvent &event);

    void SendSonarState();

    bool LoadSonar(std::string const &name);

    bool LoadImu(std::string const &name);

    void OnUpdate();

};

} // namespace gazebo

#endif
