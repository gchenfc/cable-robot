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
  // ros::Subscriber<std_msgs::Float32, RosAxis> sub_KpVel;
  // ros::Subscriber<std_msgs::Float32, RosAxis> sub_KiVel;
	ros::Subscriber<std_msgs::Float32, RosAxis> sub_KpPos;
  ros::Subscriber<std_msgs::Empty, RosAxis> sub_clearErrors;
	ros::Subscriber<std_msgs::Float32, RosAxis> sub_curLim;
	ros::Subscriber<std_msgs::Int32, RosAxis> sub_pos_est;
	ros::Publisher pub_error;
	ros::Publisher pub_cur_state;
	ros::Publisher pub_cur_pos;
	ros::Publisher pub_cur_vel;
	ros::Publisher pub_cur_cur;
	ros::Publisher pub_cur_setCur;
	ros::Publisher pub_cur_volt;
  ros::Publisher pub_lim_cur;

	int32_t posZero = 0;

	char* assembleName(char* topicPrefix, uint16_t nodeID, char* topicSuffix){
		char* result = (char*) malloc(50 * sizeof(char));
		strcpy(result, topicPrefix);
		strcat(result, itoa(nodeID, nodeIDstr, 10));
		strcat(result, topicSuffix);
		return result;
	}
public:
	int32_t last_error = Error_t::ERROR_NONE;
	int32_t last_state = State_t::AXIS_STATE_IDLE;
	float last_pos = 0;
	float last_vel = 0;
	float last_cur = 0;
	float last_setCur = 0;
	float last_volt = 0;
  float last_curLim = 0;

	RosAxis(uint16_t nodeID) :
		nodeID_(nodeID),
		sub_estop(assembleName("odrv/axis", nodeID, "/estop"), &RosAxis::sub_estop_cb, this),
		sub_ctrl(assembleName("odrv/axis", nodeID, "/activate"), &RosAxis::sub_ctrl_cb, this),
		sub_pos(assembleName("odrv/axis", nodeID, "/set_pos"), &RosAxis::sub_pos_cb, this),
		sub_cur(assembleName("odrv/axis", nodeID, "/set_cur"), &RosAxis::sub_cur_cb, this),
    sub_KpPos(assembleName("odrv/axis", nodeID, "/set_KpPos"), &RosAxis::sub_KpPos_cb, this),
    sub_clearErrors(assembleName("odrv/axis", nodeID, "/clear_errors"), &RosAxis::sub_clearErrors_cb, this),
    sub_curLim(assembleName("odrv/axis", nodeID, "/set_lim_cur"), &RosAxis::sub_curLim_cb, this),
    sub_pos_est(assembleName("odrv/axis", nodeID, "/set_pos_est"), &RosAxis::sub_pos_est_cb, this),
		pub_error(assembleName("odrv/axis", nodeID, "/error"), &outMsg32),
		pub_cur_state(assembleName("odrv/axis", nodeID, "/cur_state"), &outMsg32),
		pub_cur_pos(assembleName("odrv/axis", nodeID, "/cur_pos"), &outMsgFloat),
		pub_cur_vel(assembleName("odrv/axis", nodeID, "/cur_vel"), &outMsgFloat),
		pub_cur_cur(assembleName("odrv/axis", nodeID, "/cur_cur"), &outMsgFloat),
		pub_cur_setCur(assembleName("odrv/axis", nodeID, "/cur_setCur"), &outMsgFloat),
		pub_cur_volt(assembleName("odrv/axis", nodeID, "/cur_volt"), &outMsgFloat),
    pub_lim_cur(assembleName("odrv/axis", nodeID, "/lim_cur"), &outMsgFloat)
	{};

	void init() {
		nh.subscribe(this->sub_estop);
		nh.subscribe(this->sub_ctrl);
		nh.subscribe(this->sub_pos);
		nh.subscribe(this->sub_cur);
    nh.subscribe(this->sub_KpPos);
    nh.subscribe(this->sub_clearErrors);
    nh.subscribe(this->sub_curLim);
    nh.subscribe(this->sub_pos_est);
		nh.advertise(pub_error);
		nh.advertise(pub_cur_state);
		nh.advertise(pub_cur_pos);
		nh.advertise(pub_cur_vel);
		nh.advertise(pub_cur_cur);
		nh.advertise(pub_cur_setCur);
		nh.advertise(pub_cur_volt);
    nh.advertise(pub_lim_cur);
	};
	
	void sub_estop_cb(const std_msgs::Empty& estop){
		set_estop();
	};
	void sub_ctrl_cb(const std_msgs::Bool& ctrl){
		set_ctrl(ctrl.data);
	};
	void sub_pos_cb(const std_msgs::Int32& pos){
		set_pos(pos.data, 0, 0);
	};
	void sub_cur_cb(const std_msgs::Float32& cur){
		set_cur(cur.data);
	};
	void sub_KpVel_cb(const std_msgs::Float32& data){
    return; // TODO(gerry): this and KiVel should be bundled together
  }
	void sub_KiVel_cb(const std_msgs::Float32& data){
    return; // TODO(gerry)
  }
	void sub_KpPos_cb(const std_msgs::Float32& data){
    set_KpPos(data.data);
  }
  void sub_clearErrors_cb(const std_msgs::Empty& data){
    set_clearErrors();
  }
	void sub_curLim_cb(const std_msgs::Float32& data){
    set_curLim(data.data);
  }
	void sub_pos_est_cb(const std_msgs::Int32& data){
    set_pos_est(data.data);
  }


	// void set_KpVel(float data){
  //   sendMsg(nodeID_,  , data)
  // }
	// void set_KiVel(float data){
  //   sendMsg(nodeID_, MSG_ , data)
  // }
	void set_KpPos(float data){
    sendMsg(nodeID_, MSG_SET_POSITION_GAIN, data);
  }
  void set_clearErrors(){
    sendMsg(nodeID_, MSG_CLEAR_ERROR, (int32_t)0);
  }
	void set_curLim(float data){
    sendMsg(nodeID_, MSG_SET_CURRENT_LIMIT, data);
  }
	void set_pos_est(int32_t encoder_pos){
    sendMsg(nodeID_, MSG_SET_LINEAR_COUNT, encoder_pos);
  }
	void set_estop(){
		sendMsg(nodeID_, MSG_ODRIVE_ESTOP, (int32_t)0);
	};
	void set_ctrl(bool ctrl){
		sendMsg(nodeID_, MSG_SET_AXIS_REQUESTED_STATE,
            (int32_t)(ctrl ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE));
	};
	void set_pos(int32_t pos, float vel_FF, float cur_FF){
		sendMsg(nodeID_, MSG_SET_POS_SETPOINT, convertPosToLocal(pos),
											   static_cast<int32_t>(vel_FF*10),
											   static_cast<int32_t>(cur_FF*100));
	};
	void set_vel(float vel, float cur_FF=0){
		sendMsg(nodeID_, MSG_SET_VEL_SETPOINT, static_cast<int32_t>(vel*100),
											   static_cast<int32_t>(cur_FF*100));
	};
	void set_cur(float cur){
		sendMsg(nodeID_, MSG_SET_CUR_SETPOINT, static_cast<int32_t>(cur*100));
	};

	void readCAN(CAN_message_t inMsg) {
    switch(inMsg.id & 0x1F) {
      case CanMsg_t::MSG_ODRIVE_HEARTBEAT:
        publishError(*(int32_t*)(inMsg.buf));
        publishCurState(*(int32_t*)(&inMsg.buf[4]));
        break;
      case CanMsg_t::MSG_GET_ENCODER_ESTIMATES:
        // float32_t does not exist :(
        publishCurPos(convertPosToGlobal(*(float*)(inMsg.buf)));
        publishCurVel(*(float*)(&inMsg.buf[4]));
        break;
      case MSG_GET_IQ:
        publishCurSetCur(*(float*)(&inMsg.buf));
        publishCurCur(*(float*)(&inMsg.buf[4]));
        break;
      case MSG_GET_VBUS_VOLTAGE:
        publishCurVolt(*(float*)(&inMsg.buf));
        break;
      case MSG_GET_CURRENT_LIMIT:
        publishCurLim(*(float*)(&inMsg.buf));
        break;
    }
	}
	void publishError(int32_t val){
		last_error = val;
		outMsg32.data = val;
		pub_error.publish(&outMsg32);
	}
	void publishCurState(int32_t val){
		last_state = val;
		outMsg32.data = val;
		pub_cur_state.publish(&outMsg32);
	}
	void publishCurPos(float val) {
		last_pos = val;
		outMsgFloat.data = val;
		pub_cur_pos.publish(&outMsgFloat);
	}
	void publishCurVel(float val) {
		last_vel = val;
		outMsgFloat.data = val;
		pub_cur_vel.publish(&outMsgFloat);
	}
	void publishCurCur(float val) {
		last_cur = val;
		outMsgFloat.data = val;
		pub_cur_cur.publish(&outMsgFloat);
	}
	void publishCurSetCur(float val) {
		last_setCur = val;
		outMsgFloat.data = val;
		pub_cur_setCur.publish(&outMsgFloat);
	}
	void publishCurVolt(float val) {
		last_volt = val;
		outMsgFloat.data = val;
		pub_cur_volt.publish(&outMsgFloat);
	}
  void publishCurLim(float val) {
    last_curLim = val;
    outMsgFloat.data = val;
    pub_lim_cur.publish(&outMsgFloat);
  }

  void setPosZero(void) {
    posZero += last_pos;
  }
	int32_t convertPosToGlobal(int32_t pos) {
		return pos - posZero;
	}
	int32_t convertPosToLocal(int32_t pos) {
		return pos + posZero;
	}
};