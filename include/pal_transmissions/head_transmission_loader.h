/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef TRANSMISSIONS_HEAD__TRANSMISSION_LOADER_H
#define TRANSMISSIONS_HEAD__TRANSMISSION_LOADER_H

// C++ standard
#include <string>

// ros_control
#include <transmission_interface/transmission_loader.h>

// Current package
#include <pal_transmissions/head_transmission.h>

namespace pal_transmissions
{

/**
 * \brief Class for loading a simple transmission instance from configuration data.
 */
class HeadTransmissionLoader : public transmission_interface::TransmissionLoader
{
public:
  typedef transmission_interface::TransmissionLoader::TransmissionPtr TransmissionPtr;
  typedef transmission_interface::TransmissionInfo TransmissionInfo;

  TransmissionPtr load(const TransmissionInfo& transmission_info);

private:
  typedef std::vector<HeadTransmission::Limits> LimitsVec;

  static bool getActuatorConfig(const TransmissionInfo& transmission_info,
                                std::vector<double>&    actuator_reduction);

  static bool getJointConfig(const TransmissionInfo& transmission_info,
                             std::vector<double>&    joint_offset,
                             LimitsVec&              limits_vec);

  static ParseStatus parseJointValue(const TiXmlElement& joint_el,
                                     const std::string&  value_name,
                                     const std::string&  joint_name,
                                     const std::string&  transmission_name,
                                     double&             value);

  static ParseStatus parseLimits(const TiXmlElement& joint_el,
                                 const std::string&  joint_name,
                                 const std::string&  transmission_name,
                                 LimitsVec&          limits_vec);
};

} // namespace

#endif // header guard
