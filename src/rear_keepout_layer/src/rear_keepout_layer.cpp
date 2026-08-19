#include "rear_keepout_layer/rear_keepout_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rear_keepout_layer
{

void RearKeepoutLayer::declareParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RearKeepoutLayer: lifecycle node lock failed");
  }

  const auto declare_if_missing = [&node](const std::string & key, const auto & default_value) {
      if (!node->has_parameter(key)) {
        node->declare_parameter(key, default_value);
      }
    };

  declare_if_missing(name_ + ".enabled", true);
  declare_if_missing(name_ + ".road_width", 6.0);
  declare_if_missing(name_ + ".vehicle_rear_x", -1.35);
  declare_if_missing(name_ + ".rear_gap", 0.80);
  declare_if_missing(name_ + ".rear_bar_thickness", 0.40);
  declare_if_missing(name_ + ".side_arm_thickness", 0.35);
  declare_if_missing(name_ + ".arm_front_x", 1.80);
  declare_if_missing(name_ + ".bounds_padding", 0.20);
}

void RearKeepoutLayer::refreshParameters()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".road_width", road_width_);
  node->get_parameter(name_ + ".vehicle_rear_x", vehicle_rear_x_);
  node->get_parameter(name_ + ".rear_gap", rear_gap_);
  node->get_parameter(name_ + ".rear_bar_thickness", rear_bar_thickness_);
  node->get_parameter(name_ + ".side_arm_thickness", side_arm_thickness_);
  node->get_parameter(name_ + ".arm_front_x", arm_front_x_);
  node->get_parameter(name_ + ".bounds_padding", bounds_padding_);

  // Calisma sirasinda hatali parametre girilirse layer'in geometrisini bozmak yerine
  // guvenli minimumlara clamp et.
  road_width_ = std::max(road_width_, 1.0);
  rear_gap_ = std::max(rear_gap_, 0.10);
  rear_bar_thickness_ = std::max(rear_bar_thickness_, 0.10);
  side_arm_thickness_ = std::clamp(side_arm_thickness_, 0.10, road_width_ * 0.45);
  bounds_padding_ = std::max(bounds_padding_, 0.0);

  // U'nun yan kollari en azindan arac merkezinin biraz onune kadar gelsin.
  arm_front_x_ = std::max(arm_front_x_, 0.0);
}

void RearKeepoutLayer::onInitialize()
{
  declareParameters();
  refreshParameters();

  current_ = true;
  pose_valid_ = false;
  have_previous_bounds_ = false;

  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(
      node->get_logger(),
      "RearKeepoutLayer active: road_width=%.2f m, vehicle_rear_x=%.2f m, "
      "rear_gap=%.2f m, rear_bar=%.2f m, side_arm=%.2f m, arm_front_x=%.2f m",
      road_width_, vehicle_rear_x_, rear_gap_, rear_bar_thickness_,
      side_arm_thickness_, arm_front_x_);
  }
}

bool RearKeepoutLayer::pointInsideKeepout(double x_robot, double y_robot) const
{
  const double half_width = road_width_ * 0.5;

  // Aracin arka tamponu vehicle_rear_x_. Duvar tamponun rear_gap_ kadar arkasinda baslar.
  const double bar_front_x = vehicle_rear_x_ - rear_gap_;
  const double bar_rear_x = bar_front_x - rear_bar_thickness_;

  const double abs_y = std::abs(y_robot);

  // U'nun yatay arka parcasi: yolun tam 6 m genisligini kapatir.
  const bool in_rear_bar =
    x_robot >= bar_rear_x && x_robot <= bar_front_x && abs_y <= half_width;

  // U'nun iki yan kolu: yol kenarlarinda ince seritler.
  // On taraf aciktir; normal ileri rota bu acikliktan cikar.
  const bool in_side_arms =
    x_robot >= bar_rear_x && x_robot <= arm_front_x_ &&
    abs_y >= (half_width - side_arm_thickness_) && abs_y <= half_width;

  return in_rear_bar || in_side_arms;
}

void RearKeepoutLayer::computeCurrentBounds(
  double robot_x, double robot_y, double robot_yaw,
  double & min_x, double & min_y, double & max_x, double & max_y) const
{
  const double half_width = road_width_ * 0.5;
  const double bar_front_x = vehicle_rear_x_ - rear_gap_;
  const double bar_rear_x = bar_front_x - rear_bar_thickness_;

  // U'nun kapsadigi en dis dikdortgenin 4 kosesi. Gercek cost sadece U hucrelerine yazilir.
  const std::array<std::array<double, 2>, 4> corners{{
    {{bar_rear_x, -half_width}},
    {{bar_rear_x,  half_width}},
    {{arm_front_x_, -half_width}},
    {{arm_front_x_,  half_width}}
  }};

  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);

  min_x = std::numeric_limits<double>::infinity();
  min_y = std::numeric_limits<double>::infinity();
  max_x = -std::numeric_limits<double>::infinity();
  max_y = -std::numeric_limits<double>::infinity();

  for (const auto & p : corners) {
    const double wx = robot_x + c * p[0] - s * p[1];
    const double wy = robot_y + s * p[0] + c * p[1];

    min_x = std::min(min_x, wx);
    min_y = std::min(min_y, wy);
    max_x = std::max(max_x, wx);
    max_y = std::max(max_y, wy);
  }

  min_x -= bounds_padding_;
  min_y -= bounds_padding_;
  max_x += bounds_padding_;
  max_y += bounds_padding_;
}

void RearKeepoutLayer::expandBounds(
  double src_min_x, double src_min_y, double src_max_x, double src_max_y,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  *min_x = std::min(*min_x, src_min_x);
  *min_y = std::min(*min_y, src_min_y);
  *max_x = std::max(*max_x, src_max_x);
  *max_y = std::max(*max_y, src_max_y);
}

void RearKeepoutLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  refreshParameters();

  // Onceki U konumunu da update penceresine sok. LayeredCostmap bu bolgeyi sifirlayip
  // alttaki static/obstacle/inflation/middle-lane katmanlarini yeniden uygular; boylece
  // hareket eden duvar haritada iz birakmaz.
  if (have_previous_bounds_) {
    expandBounds(
      previous_min_x_, previous_min_y_, previous_max_x_, previous_max_y_,
      min_x, min_y, max_x, max_y);
  }

  if (!enabled_) {
    // Eski alan bu turda temizlensin, sonraki turlarda gereksiz yere buyuk bounds istemeyelim.
    have_previous_bounds_ = false;
    pose_valid_ = false;
    current_ = true;
    return;
  }

  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;
  pose_valid_ = true;

  double cur_min_x, cur_min_y, cur_max_x, cur_max_y;
  computeCurrentBounds(
    robot_x_, robot_y_, robot_yaw_, cur_min_x, cur_min_y, cur_max_x, cur_max_y);

  expandBounds(cur_min_x, cur_min_y, cur_max_x, cur_max_y, min_x, min_y, max_x, max_y);

  previous_min_x_ = cur_min_x;
  previous_min_y_ = cur_min_y;
  previous_max_x_ = cur_max_x;
  previous_max_y_ = cur_max_y;
  have_previous_bounds_ = true;
  current_ = true;
}

void RearKeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !pose_valid_) {
    return;
  }

  const double c = std::cos(robot_yaw_);
  const double s = std::sin(robot_yaw_);

  // Bu plugin YAML'da en SONDA tutulur. Bilerek master_grid'e direkt LETHAL yaziyoruz.
  // Boylece unknown(255) hucrelerde dahi bariyer LETHAL(254) olur ve inflation layer
  // bu sentetik U'yu arac tarafina sisirmez.
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      if (i < 0 || j < 0 ||
        i >= static_cast<int>(master_grid.getSizeInCellsX()) ||
        j >= static_cast<int>(master_grid.getSizeInCellsY()))
      {
        continue;
      }

      double wx, wy;
      master_grid.mapToWorld(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);

      const double dx = wx - robot_x_;
      const double dy = wy - robot_y_;

      // map/global -> base_footprint lokal koordinati (inverse SE2)
      const double x_robot = c * dx + s * dy;
      const double y_robot = -s * dx + c * dy;

      if (pointInsideKeepout(x_robot, y_robot)) {
        master_grid.setCost(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j),
          nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void RearKeepoutLayer::reset()
{
  pose_valid_ = false;
  have_previous_bounds_ = false;
  current_ = true;
}

}  // namespace rear_keepout_layer

PLUGINLIB_EXPORT_CLASS(rear_keepout_layer::RearKeepoutLayer, nav2_costmap_2d::Layer)
