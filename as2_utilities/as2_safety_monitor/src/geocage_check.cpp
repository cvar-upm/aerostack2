// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file geocage_check.cpp
*
* Geocage check source file.
*
* @author Javilinos
*/

#include "as2_safety_monitor/geocage_check.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "as2_core/names/topics.hpp"
#include "as2_safety_monitor/pnpoly.hpp"
#include "tf2/exceptions.h"
#include "yaml-cpp/yaml.h"

namespace as2_safety_monitor
{

std::vector<std::array<double, 2>> toPolygon(const std::string & text)
{
  std::vector<std::array<double, 2>> polygon;
  try {
    const YAML::Node vertices = YAML::Load(text);
    if (!vertices.IsSequence()) {
      throw std::invalid_argument("polygon must be a list of [x, y] pairs");
    }
    for (const auto & vertex : vertices) {
      if (!vertex.IsSequence() || vertex.size() != 2) {
        throw std::invalid_argument("polygon vertices must be [x, y] pairs");
      }
      const std::array<double, 2> point{vertex[0].as<double>(), vertex[1].as<double>()};
      if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
        throw std::invalid_argument("polygon vertices must be finite");
      }
      polygon.push_back(point);
    }
  } catch (const YAML::Exception & ex) {
    throw std::invalid_argument("polygon is not a list of [x, y] pairs: " + std::string(ex.what()));
  }
  if (polygon.size() < 3) {
    throw std::invalid_argument("polygon must list at least three [x, y] vertices");
  }
  return polygon;
}

bool isInsideCage(
  std::vector<std::array<double, 2>> & polygon, double z_min, double z_max,
  const geometry_msgs::msg::Point & position)
{
  std::array<double, 2> point{position.x, position.y};
  return position.z >= z_min && position.z <= z_max && Pnpoly::isIn(polygon, point);
}

GeocageCheck::GeocageCheck(
  as2::Node * node, const std::string & name, const AlertPublisher::SharedPtr & alert_pub)
: SafetyCheck(node, name, as2_names::topics::self_localization::qos, alert_pub),
  frame_id_(as2::tf::generateTfName(node, node->getParameter<std::string>(name + ".frame_id"))),
  polygon_(toPolygon(node->getParameter<std::string>(name + ".polygon"))),
  z_min_(node->getParameter<double>(name + ".z_min")),
  z_max_(node->getParameter<double>(name + ".z_max")),
  tf_handler_(node)
{
  if (!(z_min_ < z_max_)) {
    throw std::invalid_argument(name + ".z_min must be lower than " + name + ".z_max");
  }
}

std::optional<bool> GeocageCheck::isViolated(const geometry_msgs::msg::PoseStamped & msg)
{
  if (msg.header.frame_id == frame_id_) {
    return !isInsideCage(polygon_, z_min_, z_max_, msg.pose.position);
  }
  try {
    // Latest transform, never blocking the timer
    const geometry_msgs::msg::PoseStamped pose =
      tf_handler_.convert(msg, frame_id_, std::chrono::nanoseconds::zero());
    return !isInsideCage(polygon_, z_min_, z_max_, pose.pose.position);
  } catch (const tf2::TransformException & ex) {
    warnThrottled(ex.what());
    return std::nullopt;
  }
}

std::string GeocageCheck::describe(const geometry_msgs::msg::PoseStamped & msg) const
{
  char text[192];
  std::snprintf(
    text, sizeof(text), "position (%.2f, %.2f, %.2f) in %s outside the cage in %s",
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.frame_id.c_str(),
    frame_id_.c_str());
  return text;
}

}  // namespace as2_safety_monitor
