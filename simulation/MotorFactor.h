/**
 *  @file MotorFactor.h
 *  @author Zhangqi Luo, Gerry Chen and Frank Dellaert
 **/
/*
formula:
if -0.4 ~ 0.4, 0
if < -0.4, 0-> -0.4, -70 -> -2.5
if > 0.4, 0-> 0.4, 60 -> 2.5
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

class MotorFactor : public gtsam::NoiseModelFactor2<double, double> {

    private:
    	typedef MotorFactor This;
  		typedef gtsam::NoiseModelFactor2<double,double> Base;

    public:
		MotorFactor(gtsam::Key torque, gtsam::Key current,
              const gtsam::noiseModel::Base::shared_ptr &cost_model)
      			: Base(cost_model, torque, current) {}
  		virtual ~MotorFactor() {}


    	gtsam::Vector evaluateError(
	      const double &torque, const double &current, 
	      boost::optional<gtsam::Matrix &> H_torque = boost::none,
	      boost::optional<gtsam::Matrix &> H_current = boost::none) const override {
	      	gtsam::Matrix11 out;
	      	gtsam::Matrix11 partial_deriv_torque;
	      	
	      	gtsam::Matrix11 partial_deriv_current;

		    double expected = 0;
		    if (current < 0.4 && current > -0.4) {
		    	expected = 0;
		    	partial_deriv_current << 0;
		    }else if (current <= -0.4){
		    	expected = 100.0/3.0 *(current-(-0.4));
		    	partial_deriv_current << -100.0/3.0;
		    }else{
		    	expected = 100.0/3.0 *(current-0.4);
		    	partial_deriv_current << 100.0/3.0;
		    }
		    expected -= torque;
		    out << expected;
		    partial_deriv_torque << -1;
		    if (H_torque){
		    	*H_torque = partial_deriv_torque;
		    }
		    if (H_current) {
		    	*H_current = partial_deriv_current;
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

}