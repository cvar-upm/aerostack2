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
* @file geocage_check.hpp
*
* Geocage check header file.
*
* @author Javilinos
*/

#ifndef AS2_SAFETY_MONITOR__GEOCAGE_CHECK_HPP_
#define AS2_SAFETY_MONITOR__GEOCAGE_CHECK_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "as2_core/node.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_safety_monitor/safety_check.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace as2_safety_monitor
{

/**
 * @brief Parse a YAML list of [x, y] pairs, such as "[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]".
 *
 * @param text List of at least three vertices, in order.
 * @return Polygon vertices.
 * @throw std::invalid_argument if the text is not a list of at least three finite [x, y] pairs.
 */
std::vector<std::array<double, 2>> toPolygon(const std::string & text);

/**
 * @brief Check whether a position lies inside the prism of a polygon and a height range.
 *
 * @param polygon Polygon vertices, in order. Non-const because Pnpoly::isIn requires it.
 * @param z_min Lower height limit.
 * @param z_max Upper height limit.
 * @param position Position, in the polygon frame.
 * @return True if the position is inside.
 */
bool isInsideCage(
  std::vector<std::array<double, 2>> & polygon, double z_min, double z_max,
  const geometry_msgs::msg::Point & position);

/**
 * @brief Alerts when the pose leaves a polygon prism, expressed in a given frame.
 */
class GeocageCheck : public SafetyCheck<geometry_msgs::msg::PoseStamped>
{
public:
  /**
   * @brief Construct a new GeocageCheck.
   *
   * @param node Node that owns the check.
   * @param name Check name, used as parameter prefix.
   * @param alert_pub Publisher of the alert events.
   */
  GeocageCheck(
    as2::Node * node, const std::string & name, const AlertPublisher::SharedPtr & alert_pub);

protected:
  std::optional<bool> isViolated(const geometry_msgs::msg::PoseStamped & msg) override;
  std::string describe(const geometry_msgs::msg::PoseStamped & msg) const override;

private:
  std::string frame_id_;
  std::vector<std::array<double, 2>> polygon_;
  double z_min_;
  double z_max_;
  as2::tf::TfHandler tf_handler_;
};

}  // namespace as2_safety_monitor

#endif  // AS2_SAFETY_MONITOR__GEOCAGE_CHECK_HPP_
