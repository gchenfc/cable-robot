#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/symbolic/SymbolicFactor.h>

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <fstream>

#include "CableFactor.h"
#include "JointLimitFactor.h"
#include "LinearDynamicsFactor.h"

namespace cable_robot {

class LQRFactorGraph {
 private:
  std::vector<gtsam::Key> tension, point, forces, forcesAnchor, pointEE, velEE,
      forceEE;
  std::vector<std::vector<gtsam::Key>> intermediateForces;
  std::vector<std::vector<double>> EEpath;

  gtsam::Point3 anchorPoints[4] = {
      gtsam::Point3(0, 0, 0), gtsam::Point3(1, 0, 0), gtsam::Point3(0, 1, 0),
      gtsam::Point3(1, 1, 0)};
  gtsam::Pose3 EEinitLoc;
  int mass;
  std::vector<double> tension_lim;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values linPoint;

  std::vector<double> line_space(int start, int end, int n);

  std::vector<std::vector<double>> path_builder(std::vector<double> times);

  void add_level_to_intermediateForces(
      int len, std::vector<std::vector<gtsam::Key>>& target);

  gtsam::Vector vectorBuilder(int n, int l);

 public:
  void run();
};

}  // namespace cable_robot