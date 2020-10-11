#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <fstream>

namespace cable_robot {

class PositioningFactorGraph {
 private:
  std::vector<gtsam::Key> torque, tension, current, point, forces, forcesAnchor,
      pointEE, velEE, forceEE;
  // std::vector<gtsam::Key> motor;
  std::vector<std::vector<gtsam::Key>> intermediateForces;
  std::vector<std::vector<double>> EEpath;
  const double motor_radius = 25;
  gtsam::Point3 anchorPoints[4] = {
      gtsam::Point3(0, 0, 0), gtsam::Point3(1, 0, 0), gtsam::Point3(0, 1, 0),
      gtsam::Point3(1, 1, 0)};
  gtsam::Pose3 EEinitLoc;
  int mass;
  std::vector<double> tension_lim;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values linPoint;

  /// equivalent of matlab's linspace
  std::vector<double> line_space(int start, int end, int n);

  /// generates circular path
  std::vector<std::vector<double>> path_builder(std::vector<double> times);

  /// ???
  void add_level_to_intermediateForces(
      int len, std::vector<std::vector<gtsam::Key>>& target);

  /// creates a vector of length l filled with n
  gtsam::Vector vectorBuilder(int n, int l);

 public:
  /// creates and solves an LQR-style factor graph for the cable robot
  void run();
};

}  // namespace cable_robot
