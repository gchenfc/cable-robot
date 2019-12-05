#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
std_msgs::Empty outMsgEmpty;
std_msgs::Bool outMsgBool;
std_msgs::Int32 outMsg32;
std_msgs::Float32 outMsgFloat;

class RosAxis {
private:
    uint16_t nodeID_;
    char nodeIDstr[5];
    ros::Subscriber<std_msgs::Empty, RosAxis> sub_estop;
    ros::Subscriber<std_msgs::Bool, RosAxis> sub_ctrl;
    ros::Subscriber<std_msgs::Int32, RosAxis> sub_pos;
    ros::Subscriber<std_msgs::Float32, RosAxis> sub_cur;
    ros::Publisher pub_error;
    ros::Publisher pub_cur_state;
    ros::Publisher pub_cur_pos;
    ros::Publisher pub_cur_vel;
    ros::Publisher pub_cur_cur;
    ros::Publisher pub_cur_setCur;
    ros::Publisher pub_cur_volt;

    char* assembleName(char* topicPrefix, uint16_t nodeID, char* topicSuffix){
        char* result = (char*) malloc(50 * sizeof(char));
        strcpy(result, topicPrefix);
        strcat(result, itoa(nodeID, nodeIDstr, 10));
        strcat(result, topicSuffix);
        return result;
    }
public:
    RosAxis(uint16_t nodeID) :
        nodeID_(nodeID),
        sub_estop(assembleName("odrv/axis", nodeID, "/estop"), &RosAxis::sub_estop_cb, this),
        sub_ctrl(assembleName("odrv/axis", nodeID, "/activate"), &RosAxis::sub_ctrl_cb, this),
        sub_pos(assembleName("odrv/axis", nodeID, "/set_pos"), &RosAxis::sub_pos_cb, this),
        sub_cur(assembleName("odrv/axis", nodeID, "/set_cur"), &RosAxis::sub_cur_cb, this),
        pub_error(assembleName("odrv/axis", nodeID, "/error"), &outMsg32),
        pub_cur_state(assembleName("odrv/axis", nodeID, "/cur_state"), &outMsg32),
        pub_cur_pos(assembleName("odrv/axis", nodeID, "/cur_pos"), &outMsgFloat),
        pub_cur_vel(assembleName("odrv/axis", nodeID, "/cur_vel"), &outMsgFloat),
        pub_cur_cur(assembleName("odrv/axis", nodeID, "/cur_cur"), &outMsgFloat),
        pub_cur_setCur(assembleName("odrv/axis", nodeID, "/cur_setCur"), &outMsgFloat),
        pub_cur_volt(assembleName("odrv/axis", nodeID, "/cur_volt"), &outMsgFloat)
    {};

    void init() {
        nh.subscribe(this->sub_estop);
        nh.subscribe(this->sub_ctrl);
        nh.subscribe(this->sub_pos);
        nh.subscribe(this->sub_cur);
        nh.advertise(pub_error);
        nh.advertise(pub_cur_state);
        nh.advertise(pub_cur_pos);
        nh.advertise(pub_cur_vel);
        nh.advertise(pub_cur_cur);
        nh.advertise(pub_cur_setCur);
        nh.advertise(pub_cur_volt);
    };

    void sub_estop_cb( const std_msgs::Empty& estop){
        sendMsg(nodeID_, MSG_ODRIVE_ESTOP, 0);
    };
    void sub_ctrl_cb( const std_msgs::Bool& ctrl){
        sendMsg(nodeID_, MSG_SET_AXIS_REQUESTED_STATE, ctrl.data ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE);
    };
    void sub_pos_cb( const std_msgs::Int32& pos){
        sendMsg(nodeID_, MSG_SET_POS_SETPOINT, pos.data, 0, 0);
    };
    void sub_cur_cb( const std_msgs::Float32& cur){
        sendMsg(nodeID_, MSG_SET_CUR_SETPOINT, static_cast<int32_t>(cur.data*100));
    };

    void publishError(int32_t val){
        outMsg32.data = val;
        pub_error.publish(&outMsg32);
    }
    void publishCurState(int32_t val){
        outMsg32.data = val;
        pub_cur_state.publish(&outMsg32);
    }
    void publishCurPos(float val) {
        outMsgFloat.data = val;
        pub_cur_pos.publish(&outMsgFloat);
    }
    void publishCurVel(float val) {
        outMsgFloat.data = val;
        pub_cur_vel.publish(&outMsgFloat);
    }
    void publishCurCur(float val) {
        outMsgFloat.data = val;
        pub_cur_cur.publish(&outMsgFloat);
    }
    void publishCurSetCur(float val) {
        outMsgFloat.data = val;
        pub_cur_setCur.publish(&outMsgFloat);
    }
    void publishCurVolt(float val) {
        outMsgFloat.data = val;
        pub_cur_volt.publish(&outMsgFloat);
    }
};