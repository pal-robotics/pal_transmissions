/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef TRANSMISSIONS_READ_FILE_H
#define TRANSMISSIONS_READ_FILE_H

#include <string>
#include <vector>
#include <resource_retriever/retriever.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>

namespace pal_transmissions
{

inline bool readFile(const std::string& filename, std::string& contents)
{
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource resource;

  try
  {
     resource = retriever.get("package://pal_transmissions/" + filename);
  }
  catch (resource_retriever::Exception& e)
  {
    ROS_ERROR("Failed to retrieve file: %s", e.what());
    return false;
  }

  contents.assign(resource.data.get(), resource.data.get() + resource.size);
  return true;
}

typedef transmission_interface::TransmissionInfo TransmissionInfo;
typedef transmission_interface::TransmissionParser TransmissionParser;

inline std::vector<TransmissionInfo> parseUrdf(const std::string& filename)
{
  std::vector<TransmissionInfo> infos;

  std::string urdf;
  if (!readFile(filename, urdf)) {return std::vector<TransmissionInfo>();}

  TransmissionParser parser;
  if (!parser.parse(urdf, infos)) {return std::vector<TransmissionInfo>();}
  return infos;
}

} // namespace

#endif // header guard
