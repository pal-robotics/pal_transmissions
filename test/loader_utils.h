/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef TRANSMISSIONS_LOADER_UTILS_H
#define TRANSMISSIONS_LOADER_UTILS_H

#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_loader.h>
#include "read_file.h"

using namespace transmission_interface;
typedef TransmissionLoader::TransmissionPtr TransmissionPtr;


struct TransmissionPluginLoader
{
  TransmissionPluginLoader()
    :class_loader_("transmission_interface", "transmission_interface::TransmissionLoader")
  {
  }

  boost::shared_ptr<TransmissionLoader> create(const std::string& type)
  {

    try
    {
      return class_loader_.createInstance(type);
    }
    catch(...) {return boost::shared_ptr<TransmissionLoader>();}
  }

private:
  //must keep it alive because instance destroyers need it
  pluginlib::ClassLoader<TransmissionLoader>  class_loader_;
};

#endif // header guard