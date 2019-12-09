#ifndef CABLECONTROLLER_H
#define CABLECONTROLLER_H

#include <std_msgs/UInt8.h>
#include "params.h"

std_msgs::UInt8 ctrlStateUInt8;

enum ControllerState_t : uint8_t {
    CTRL_STATE_IDLE,
    CTRL_STATE_HOME,
    CTRL_STATE_CALIBRATE,
};

void dummy(std_msgs::UInt8) {

}

class CableController {
    private:
        ControllerState_t state;
        ros::Subscriber<std_msgs::UInt8, CableController> sub_state;
        ros::Publisher pub_state;
        uint8_t home_axis = 0;
        systime_t gpTimer, stateBeginTimer;
        bool state_initialized = false;

    public:
        CableController() :
            state(CTRL_STATE_IDLE),
            sub_state("cable_controller/set_state", &CableController::setRequestedState_cb, this),
            pub_state("cable_controller/cur_state", &ctrlStateUInt8)
        {};

        void init() {
            nh.subscribe(sub_state);
            nh.advertise(pub_state);
        };

        void update() {
            switch (state) {
                case ControllerState_t::CTRL_STATE_IDLE:
                    break;
                case ControllerState_t::CTRL_STATE_HOME:
                    if (home_axis >= NUM_DRIVES)
                        break;
                    if (!state_initialized){
                        axis[home_axis].set_ctrl(true);
                        axis[home_axis].set_vel(-3 * CNTS_PER_REV);
                        state_initialized = true;
                        gpTimer = chVTGetSystemTime();
                        stateBeginTimer = chVTGetSystemTime();
                    }
                    if (axis[home_axis].last_cur > (CURRENT_LIM*0.9)) {
                        if (TIME_I2MS(chVTTimeElapsedSinceX(gpTimer)) > 1500) {
                            axis[home_axis].setPosZero();
                            axis[home_axis].set_ctrl(false); // back to idle
                            home_axis++;
                            if (home_axis < NUM_DRIVES) {
                                state_initialized = false; // hack to reset next axis
                            } else {
                                home_axis = 0;
                                state_initialized = false;
                                state = ControllerState_t::CTRL_STATE_IDLE;
                            }
                        }
                    } else {
                        gpTimer = chVTGetSystemTime();
                        if (TIME_I2MS(chVTTimeElapsedSinceX(stateBeginTimer)) > 1500) {
                            // axis[home_axis].set_ctrl(true);
                            // axis[home_axis].set_vel(-3 * CNTS_PER_REV);
                        } else if (TIME_I2MS(chVTTimeElapsedSinceX(stateBeginTimer)) > 15000) {
                            home_axis = 0;
                            state_initialized = false;
                            state = ControllerState_t::CTRL_STATE_IDLE;
                        }
                        
                    }
                    break;
                case ControllerState_t::CTRL_STATE_CALIBRATE:
                    // TODO(gerry): implement callibration routine
                    break;
            }
        };

        void publish() {
            ctrlStateUInt8.data = state;
            pub_state.publish(&ctrlStateUInt8);
        };

        void setRequestedState_cb(const std_msgs::UInt8& requestedState) {
            setRequestedState(requestedState.data);
        };
        void setRequestedState(ControllerState_t requestedState) {
            switch (state) {
                case ControllerState_t::CTRL_STATE_IDLE:
                    state = requestedState;
                    break;
            } // if in another state, do nothing
        };
};

#endif