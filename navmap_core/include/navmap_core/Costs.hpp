// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file Costs.hpp
 * @brief Values of a U8 cost layer, as in costmap_2d / Nav2 (so that bridging to them is a copy).
 */

#ifndef NAVMAP_CORE__COSTS_HPP_
#define NAVMAP_CORE__COSTS_HPP_

#include <cstdint>

namespace navmap
{

constexpr std::uint8_t NO_INFORMATION = 255;               ///< Unknown, not observed
constexpr std::uint8_t LETHAL_OBSTACLE = 254;              ///< Obstacle
constexpr std::uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;  ///< The robot would touch an obstacle
constexpr std::uint8_t MAX_NON_OBSTACLE = 252;             ///< Highest traversable cost
constexpr std::uint8_t FREE_SPACE = 0;                     ///< Known free

}  // namespace navmap

#endif  // NAVMAP_CORE__COSTS_HPP_
