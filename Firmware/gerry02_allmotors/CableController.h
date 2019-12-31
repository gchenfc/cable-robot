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

class CableController {
    private:
        ControllerState_t state;
        ros::Subscriber<std_msgs::UInt8, CableController> sub_state;
        ros::Publisher pub_state;
        uint8_t home_axis = 0;
        systime_t gpTimer, stateBeginTimer, resendTimer;
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
                        Serial1.print("about to home axis ");
                        Serial1.println(home_axis);
                        // switch (home_axis) {
                        //     case 0:
                        //         axis[0].set_ctrl(true);
                        //         axis[0].set_vel(-7 * CNTS_PER_REV);
                        //         break;
                        //     case 1:
                        //         axis[1].set_ctrl(true);
                        //         axis[1].set_vel(-7 * CNTS_PER_REV);
                        //         axis[0].set_ctrl(true);
                        //         axis[0].set_cur(0);
                        //         break;
                        //     case 2:
                        //         axis[2].set_ctrl(true);
                        //         axis[2].set_vel(-7 * CNTS_PER_REV);
                        //         axis[1].set_ctrl(true);
                        //         axis[1].set_cur(0);
                        //         chThdSleepMilliseconds(1);
                        //         // axis[0].set_ctrl(true);
                        //         // axis[0].set_vel(0.1 * CNTS_PER_REV);
                        //         break;
                        //     case 3:
                        //         axis[3].set_ctrl(true);
                        //         axis[3].set_vel(-7 * CNTS_PER_REV);
                        //         axis[2].set_ctrl(true);
                        //         axis[2].set_cur(0.5 * CNTS_PER_REV);
                        //         chThdSleepMilliseconds(1);
                        //         // axis[1].set_ctrl(true);
                        //         // axis[1].set_vel(0.1 * CNTS_PER_REV);
                        //         axis[0].set_ctrl(true);
                        //         axis[0].set_cur(0.5 * CNTS_PER_REV);
                        //         break;
                        // }
                        for (uint8_t i = 0; i < NUM_DRIVES; i++) {
                            if (i != home_axis) {
                                axis[i].set_ctrl(true);
                                axis[i].set_cur(0.5);
                            } else {
                                axis[i].set_ctrl(true);
                                axis[i].set_vel(-7 * CNTS_PER_REV);
                            }
                            chThdSleepMilliseconds(1);
                        }
                        state_initialized = true;
                        gpTimer = chVTGetSystemTime();
                        resendTimer = chVTGetSystemTime();
                        stateBeginTimer = chVTGetSystemTime();
                    }
                    if ((axis[home_axis].last_cur > (0.9 * CURRENT_LIM)) &&
                        ((axis[home_axis].last_vel/CNTS_PER_REV) > -.1)) {
                        if (TIME_I2MS(chVTTimeElapsedSinceX(gpTimer)) > 1500) {
                            axis[home_axis].setPosZero();
                            Serial1.print("successfully homed axis ");
                            Serial1.println(home_axis);
                            axis[home_axis].set_ctrl(false); // back to idle, just in case
                            home_axis++;
                            if (home_axis < NUM_DRIVES) {
                                state_initialized = false; // hack to reset next axis
                            } else {
                                for (uint8_t i = 0; i < NUM_DRIVES; i++) {
                                    axis[i].set_ctrl(false); // back to idle
                                }
                                home_axis = 0;
                                state_initialized = false;
                                state = ControllerState_t::CTRL_STATE_IDLE;
                                Serial1.println("done homing - returning to idle");
                            }
                        }
                    } else {
                        gpTimer = chVTGetSystemTime();
                        if (TIME_I2MS(chVTTimeElapsedSinceX(resendTimer)) > 1500) {
                            Serial1.println("\tresending command");
                            axis[home_axis].set_ctrl(true);
                            axis[home_axis].set_vel(-7 * CNTS_PER_REV);
                            resendTimer = chVTGetSystemTime();
                        } else
                        if (TIME_I2MS(chVTTimeElapsedSinceX(stateBeginTimer)) > 15000) {
                            Serial1.println("couldn't home - returning to idle");
                            home_axis = 0;
                            state_initialized = false;
                            for (uint8_t i = 0; i < NUM_DRIVES; i++) {
                                axis[i].set_ctrl(false); // back to idle
                            }
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