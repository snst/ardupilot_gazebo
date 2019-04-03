#ifndef SDFHELPER_HH
#define SDFHELPER_HH

using namespace gazebo;

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
      // _param = _sdf->GetElement(_name)->Get<std::string>();
      // namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    return true;
  }

  _param = _defaultValue;
  if (_verbose)
  {
    gzerr << "[NazePlugin] Please specify a value for parameter ["
      << _name << "].\n";
  }
  return false;
}

// DON'T MERGE
static std::vector<std::string> getSensorScopedName(physics::ModelPtr _model,
          const std::string &_name)
{
  std::vector<std::string> names;
  for (gazebo::physics::Link_V::const_iterator iter = _model->GetLinks().begin();
       iter != _model->GetLinks().end(); ++iter)
  {
    for (unsigned int j = 0; j < (*iter)->GetSensorCount(); ++j)
    {
        const auto sensorName = (*iter)->GetSensorName(j);
        if (sensorName.size() < _name.size())
        {
            continue;
        }
        if (sensorName.substr(
                sensorName.size()
                        - _name.size(), _name.size()) ==
                _name)
        {
            names.push_back(sensorName);
        }
    }
  }
  return names;
}

#endif