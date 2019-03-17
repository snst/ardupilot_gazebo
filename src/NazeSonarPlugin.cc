/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <memory>
#include <functional>


#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include <gazebo/sensors/SonarSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Conversions.hh>
#include <gazebo/rendering/Scene.hh>
#include <include/SelectionBuffer.hh>


#include "include/NazeSonarPlugin.hh"
#include "sensor_msgs/Range.h"


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(NazeSonarPlugin)

namespace gazebo
{
  /// \brief Obtains a parameter from sdf.
  /// \param[in] _sdf Pointer to the sdf object.
  /// \param[in] _name Name of the parameter.
  /// \param[out] _param Param Variable to write the parameter to.
  /// \param[in] _default_value Default value, if the parameter not available.
  /// \param[in] _verbose If true, gzerror if the parameter is not available.
  /// \return True if the parameter was found in _sdf, false otherwise.
  template<class T>
  bool getSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
    T &_param, const T &_defaultValue, const bool &_verbose = false)
  {
    if (_sdf->HasElement(_name))
    {
      _param = _sdf->GetElement(_name)->Get<T>();
      return true;
    }

    _param = _defaultValue;
    if (_verbose)
    {
      gzerr << "[ArduPilotPlugin] Please specify a value for parameter ["
        << _name << "].\n";
    }
    return false;
  }

  class NazeSonarPluginPrivate
  {
    public:
      sensors::SonarSensorPtr parentSensor;
      //UpdateTimer updateTimer;
      //event::ConnectionPtr updateConnection;
      ros::NodeHandlePtr nh;
      ros::Timer timer;
      ros::Publisher sonar_pub;


  };
}

/////////////////////////////////////////////////
NazeSonarPlugin::NazeSonarPlugin()
    : SensorPlugin(),
      dataPtr(new NazeSonarPluginPrivate)
{
  printf("NazeSonarPlugin\n");

  if (!ros::isInitialized())
  {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      //return;
  }
}

/////////////////////////////////////////////////
NazeSonarPlugin::~NazeSonarPlugin()
{
}

void NazeSonarPlugin::timerCallback(const ros::TimerEvent& event)
{
    gzmsg << "Hi frikandel! " << dataPtr->parentSensor->Range() << "\n";

    sensor_msgs::Range msg;
    msg.min_range = dataPtr->parentSensor->RangeMin();
    msg.max_range = dataPtr->parentSensor->RangeMax();
    msg.range = dataPtr->parentSensor->Range();
    ROS_INFO("%f", msg.range);
    dataPtr->sonar_pub.publish(msg);

}

/////////////////////////////////////////////////
void NazeSonarPlugin::Load(sensors::SensorPtr _sensor,
                                  sdf::ElementPtr _sdf)
{
  printf("NazeSonarPlugin::Load\n");


  this->dataPtr->parentSensor =
    std::dynamic_pointer_cast<sensors::SonarSensor>(_sensor);

  if (!this->dataPtr->parentSensor)
  {
    gzerr << "NazeSonarPlugin not attached to a sonar sensor\n";
    return;
  }
  else
  {
      gzmsg << "NazeSonarPlugin attached to a sonar sensor\n";
  }
  gzmsg << "vor nh\n";
  this->dataPtr->nh = boost::make_shared<ros::NodeHandle>();
  this->dataPtr->sonar_pub = this->dataPtr->nh->advertise<sensor_msgs::Range>("naze_sonar", 1000);

  gzmsg << "nach nh\n";
  this->dataPtr->timer = this->dataPtr->nh->createTimer(ros::Duration(1), &NazeSonarPlugin::timerCallback, this);
  gzmsg << "nach timer\n " << this->dataPtr->timer;
/*
  this->dataPtr->parentSensor->Sonar()->ConnectUpdate(
      std::bind(&NazeSonarPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)));  
*/
/*
  // load the fiducials
  if (_sdf->HasElement("fiducial"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("fiducial");
    while (elem)
    {
      this->dataPtr->fiducials.push_back(elem->Get<std::string>());
      elem = elem->GetNextElement("fiducial");
    }
  }
  else
  {
    gzerr << "No fidicuals specified. NazeSonarPlugin will not be run."
        << std::endl;
    return;
  }
  getSdfParam<std::string>(_sdf, "irlock_addr",
      this->dataPtr->irlock_addr, "127.0.0.1");
  getSdfParam<uint16_t>(_sdf, "irlock_port",
      this->dataPtr->irlock_port, 9005);

  this->dataPtr->parentSensor->SetActive(true);

  this->dataPtr->connections.push_back(
      this->dataPtr->parentSensor->Camera()->ConnectNewImageFrame(
      std::bind(&NazeSonarPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)));
        */
}

/////////////////////////////////////////////////
void NazeSonarPlugin::OnNewFrame(const unsigned char * /*_image*/,
    unsigned int /*_width*/, unsigned int /*_height*/, unsigned int /*_depth*/,
    const std::string &/*_format*/)
{
  printf("oNewFrame\n");
  /*
  rendering::CameraPtr camera = this->dataPtr->parentSensor->Camera();
  rendering::ScenePtr scene = camera->GetScene();

  if (!this->dataPtr->selectionBuffer)
  {
    std::string cameraName = camera->OgreCamera()->getName();
    this->dataPtr->selectionBuffer.reset(
        new rendering::SelectionBuffer(cameraName, scene->OgreSceneManager(),
        camera->RenderTexture()->getBuffer()->getRenderTarget()));
  }

  for (const auto &f : this->dataPtr->fiducials)
  {
    // check if fiducial is visible within the frustum
    rendering::VisualPtr vis = scene->GetVisual(f);
    if (!vis)
      continue;

    if (!camera->IsVisible(vis))
      continue;

    ignition::math::Vector2i pt = GetScreenSpaceCoords(
        vis->WorldPose().Pos(), camera);

    // use selection buffer to check if visual is occluded by other entities
    // in the camera view
    Ogre::Entity *entity =
      this->dataPtr->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

    rendering::VisualPtr result;
    if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
    {
      try
      {
        result = scene->GetVisual(
            Ogre::any_cast<std::string>(
            entity->getUserObjectBindings().getUserAny()));
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
        continue;
      }
    }

    if (result && result->GetRootVisual() == vis)
    {
    }
  }
  */
}

