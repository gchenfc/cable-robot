/**
 * @file WinchFactor.h
 * @brief Maps shaft torque to cable tension
 * @author Zhangqi Luo, Gerry Chen and Frank Dellaert
 * 
 * Applies a linear mapping from shaft torque to cable tension based on radius
 **/
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

namespace cable_robot {

class WinchFactor : public gtsam::NoiseModelFactor2<double, double> {
 private:
  typedef WinchFactor This;
  typedef gtsam::NoiseModelFactor2<double, double> Base;
  double radius;

 public:
  WinchFactor(gtsam::Key tension, gtsam::Key torque,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               double ra = 25.0)
      : Base(cost_model, tension, torque), radius(ra) {}
  virtual ~WinchFactor() {}

  gtsam::Vector evaluateError(
      const double &tension, const double &torque,
      boost::optional<gtsam::Matrix &> H_tension = boost::none,
      boost::optional<gtsam::Matrix &> H_torque = boost::none) const override {
    gtsam::Matrix11 out;
    gtsam::Matrix11 partial_deriv_torque;

    gtsam::Matrix11 partial_deriv_tension;

    double expected = torque * radius;

    expected -= tension;
    out << expected;

    partial_deriv_torque << radius;
    partial_deriv_tension << -1;

    if (H_torque) {
      *H_torque = partial_deriv_torque;
    }
    if (H_tension) {
      *H_tension = partial_deriv_tension;
    }

    return out;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace cable_robot
