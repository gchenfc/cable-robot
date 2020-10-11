/**
 *  @file  LinearDynamicsFactor.h
 *  @brief Linear Dynamics in Point3
 *  @author Gerry Chen and Frank Dellaert
 **/
#pragma once

#include "util/utils.h"

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>

namespace manipulator {

/**
 * 6-way factor for Gaussian Process prior factor, linear version
 * Dynamic model:
 * x_plus = Phi * x + noise
 * error:
 * Phi * x - x_plus
 */
class LinearDynamicsFactor
    : public gtsam::NoiseModelFactor5<gtsam::Point3, gtsam::Vector3, gtsam::Vector3,
                                      gtsam::Point3, gtsam::Vector3> {
 private:
  double delta_t_;
  gtsam::Matrix A_;
  gtsam::Matrix B_;

  typedef LinearDynamicsFactor This;
  typedef gtsam::NoiseModelFactor5<gtsam::Point3, gtsam::Vector3, gtsam::Vector3,
                                   gtsam::Point3, gtsam::Vector3>
      Base;

 public:
  LinearDynamicsFactor() {
  } /* Default constructor only for serialization */

  /** Constructor
    delta_t is the time between the two states
  */
  LinearDynamicsFactor(
      gtsam::Key q_key1, gtsam::Key qVel_key1, gtsam::Key qAccel_key1,
      gtsam::Key q_key2, gtsam::Key qVel_key2,
      const gtsam::noiseModel::Gaussian::shared_ptr &Qc_model, double delta_t)
      : Base(Qc_model,
             q_key1, qVel_key1, qAccel_key1, q_key2, qVel_key2),
        delta_t_(delta_t),
         A_(gtsam::Matrix(6, 6)), 
         B_(gtsam::Matrix(6, 3)){
    double tau = delta_t_;
    //A_ = gtsam::Matrix(6, 6);
    A_ << gtsam::I_3x3, gtsam::I_3x3 * tau,
          gtsam::Z_3x3, gtsam::I_3x3;
    //B_ = gtsam::Matrix(6, 3);
    B_ << gtsam::I_3x3 * 0.5*tau*tau,   gtsam::I_3x3 * tau;

  }

  virtual ~LinearDynamicsFactor() {}

  /** evaluate Gaussian process errors
    Keyword argument:
      q1         -- joint coordinates at time 1
      qVel1      -- joint velocity at time 1
      qAccel1    -- joint acceleration at time 1
      q2         -- joint coordinates at time 2
      qVel2      -- joint velocity at time 2
      qAccel2    -- joint acceleration at time 2
    */
  gtsam::Vector evaluateError(
      const gtsam::Point3 &q1, const gtsam::Vector3 &qVel1, const gtsam::Vector3 &qAccel1,
      const gtsam::Point3 &q2, const gtsam::Vector3 &qVel2,
      boost::optional<gtsam::Matrix &> H_q1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_q2 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel2 = boost::none) const override {
    double tau = delta_t_;
    // state vector
    gtsam::Vector6 x1, x2;
    x1 << q1.vector(), qVel1;
    x2 << q2.vector(), qVel2;

    // Jacobians
    if (H_q1) *H_q1 = (gtsam::Matrix(6, 3) << -gtsam::I_3x3,
                                               gtsam::Z_3x3)
                                            .finished();
    if (H_qVel1) *H_qVel1 = (gtsam::Matrix(6, 3) << -gtsam::I_3x3 * tau,
                                                    -gtsam::I_3x3)
                                                .finished();
    if (H_qAccel1) *H_qAccel1 = (gtsam::Matrix(6, 3) << -gtsam::I_3x3 * 0.5*tau*tau,
                                                        -gtsam::I_3x3 * tau)
                                                    .finished();
    if (H_q2) *H_q2 = (gtsam::Matrix(6,3) << gtsam::I_3x3, gtsam::Z_3x3).finished();
    if (H_qVel2) *H_qVel2 = (gtsam::Matrix(6,3) << gtsam::Z_3x3, gtsam::I_3x3).finished();
    // transition matrix & error
    return x2 - A_ * x1 - B_ * qAccel1;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** equals specialized to this factor */
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol) &&
           fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "6-way Gaussian Process Factor Linear" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(delta_t_);
  }
};  // LinearDynamicsFactor

}  // namespace manipulator

/// traits
namespace gtsam {
template <>
struct traits<manipulator::LinearDynamicsFactor>
    : public Testable<manipulator::LinearDynamicsFactor> {};
}  // namespace gtsam
