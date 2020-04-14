/**
 * @file  CableFactor.h
 * @brief Cable tension factor, relates cable tension, two mounting points, and resultant forces
 * @Author: Frank Dellaert and Gerry Chen
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Unit3.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

namespace manipulator {

/** CableFactor is a 5-way nonlinear factor which enforces relation amongst cable tension, mounting
 * points, and forces felt by the mounting points */
class CableFactor : public gtsam::NoiseModelFactor5<double, gtsam::Point3, gtsam::Point3,
                                                    gtsam::Vector, gtsam::Vector> {
 private:
  typedef CableFactor This;
  typedef gtsam::NoiseModelFactor5<double, gtsam::Point3, gtsam::Point3,
                                   gtsam::Vector, gtsam::Vector> Base;
  //int[] segmentation;
 public:
  /** Cable factor
      Arguments:
          tension_key -- key for cable tension
          point1_key -- key for attachment point 1
          point2_key -- key for attachment point 2
          force1_key -- key for force on attachment point 1
          force2_key -- key for force on attachment point 2
          cost_model -- noise model (6 dimensional: 3 force directions for 2 attachment points)
   */
  CableFactor(gtsam::Key tension_key, gtsam::Key point1_key, gtsam::Key point2_key,
              gtsam::Key force1_key, gtsam::Key force2_key,
              const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, tension_key, point1_key, point2_key, force1_key, force2_key) {}
  virtual ~CableFactor() {}

 public:
  /** Cable factor
      Arguments:
          tension_key -- key for cable tension
          point1_key -- key for attachment point 1
          point2_key -- key for attachment point 2
          force1_key -- key for force on attachment point 1
          force2_key -- key for force on attachment point 2
   */
  gtsam::Vector evaluateError(
      const double &tension, const gtsam::Point3 &point1, const gtsam::Point3 &point2,
      const gtsam::Vector &force1, const gtsam::Vector &force2,
      boost::optional<gtsam::Matrix &> H_tension = boost::none,
      boost::optional<gtsam::Matrix &> H_point1 = boost::none,
      boost::optional<gtsam::Matrix &> H_point2 = boost::none,
      boost::optional<gtsam::Matrix &> H_force1 = boost::none,
      boost::optional<gtsam::Matrix &> H_force2 = boost::none) const override {
    gtsam::Matrix33 H_dir_Dir;
    gtsam::Point3 dir = gtsam::normalize(point2 - point1, H_dir_Dir);
    gtsam::Vector6 error;
    error << tension*dir - force1,
            -tension*dir - force2;

    gtsam::Matrix63 H_err_Dir;
    H_err_Dir << tension*H_dir_Dir, -tension*H_dir_Dir;

    // std::cout << tension << '\t' << dir << '\t' << H_dir_Dir << std::endl;

    if (H_tension) {
      gtsam::Matrix61 H_tension_;
      H_tension_ << dir, -dir;
      *H_tension = H_tension_;
    }
    if (H_point1) {
      *H_point1 = -H_err_Dir;
    }
    if (H_point2) {
      *H_point2 = H_err_Dir;
    }
    if (H_force1) {
      gtsam::Matrix63 H_force1_;
      H_force1_ << -gtsam::I_3x3, gtsam::Z_3x3;
      *H_force1 = H_force1_;
    }
    if (H_force2) {
      gtsam::Matrix63 H_force2_;
      H_force2_ << gtsam::Z_3x3, -gtsam::I_3x3;
      *H_force2 = H_force2_;
    }

    return error;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "cable factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};

}