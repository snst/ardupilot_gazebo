#include "RotorControl.hh"

double RotorControl::kDefaultRotorVelocitySlowdownSim = 10.0;
double RotorControl::kDefaultFrequencyCutoff = 5.0;
double RotorControl::kDefaultSamplingRate = 0.2;

RotorControl::RotorControl()
{
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
}
