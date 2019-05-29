/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef HALF_TRANSMISSION_INTERFACE_DIFFERENTIAL_TRANSMISSION_LOADER_H
#define HALF_TRANSMISSION_INTERFACE_DIFFERENTIAL_TRANSMISSION_LOADER_H

// TinyXML
#include <tinyxml.h>

// ros_control
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface
{

/**
 * \brief TODO
 *
 */
class HalfDifferentialTransmissionLoader : public TransmissionLoader
{
public:
  TransmissionPtr load(const TransmissionInfo& transmission_info);

private:
  static bool getActuatorConfig(const TransmissionInfo& transmission_info,
                                std::vector<double>&    actuator_reduction);

  static bool getJointConfig(const TransmissionInfo& transmission_info,
                             std::vector<double>&    joint_reduction,
                             std::vector<double>&    joint_offset);
};

} // namespace

#endif // header guard
