// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, softwact_reduction_e
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT Wact_reduction_RANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HALF_DIFFERENTIAL_TRANSMISSION_LOADER_H
#define HALF_DIFFERENTIAL_TRANSMISSION_LOADER_H

#include "transmission_interface/transmission_loader.hpp"

namespace pal_transmissions
{

class HalfDifferentialTransmissionLoader : public transmission_interface::TransmissionLoader
{
public:
  std::shared_ptr<transmission_interface::Transmission> load(
    const hardware_interface::TransmissionInfo & transmission_info) override;
};

} // namespace pal_transmissions

#endif // HALF_DIFFERENTIAL_TRANSMISSION_LOADER_H
