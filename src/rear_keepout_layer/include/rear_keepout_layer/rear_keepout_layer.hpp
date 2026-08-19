#ifndef REAR_KEEPOUT_LAYER__REAR_KEEPOUT_LAYER_HPP_
#define REAR_KEEPOUT_LAYER__REAR_KEEPOUT_LAYER_HPP_

#include <string>

#include "nav2_costmap_2d/layer.hpp"

namespace rear_keepout_layer
{

class RearKeepoutLayer : public nav2_costmap_2d::Layer
{
public:
  RearKeepoutLayer() = default;
  ~RearKeepoutLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  bool isClearable() override
  {
    return false;
  }

private:
  void declareParameters();
  void refreshParameters();

  bool pointInsideKeepout(double x_robot, double y_robot) const;

  void computeCurrentBounds(
    double robot_x, double robot_y, double robot_yaw,
    double & min_x, double & min_y, double & max_x, double & max_y) const;

  static void expandBounds(
    double src_min_x, double src_min_y, double src_max_x, double src_max_y,
    double * min_x, double * min_y, double * max_x, double * max_y);

  // U geometrisi base_footprint koordinatinda tanimlanir.
  // +X ileri, +Y sol.
  double road_width_{6.0};
  double vehicle_rear_x_{-1.35};
  double rear_gap_{0.80};
  double rear_bar_thickness_{0.40};
  double side_arm_thickness_{0.35};
  double arm_front_x_{1.80};
  double bounds_padding_{0.20};

  // updateCosts() icin son robot pozu
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  bool pose_valid_{false};

  // Hareket eden duvarin eski konumunun master costmap'ten temizlenebilmesi icin
  // bir onceki AABB updateBounds() icine tekrar dahil edilir.
  bool have_previous_bounds_{false};
  double previous_min_x_{0.0};
  double previous_min_y_{0.0};
  double previous_max_x_{0.0};
  double previous_max_y_{0.0};
};

}  // namespace rear_keepout_layer

#endif  // REAR_KEEPOUT_LAYER__REAR_KEEPOUT_LAYER_HPP_
