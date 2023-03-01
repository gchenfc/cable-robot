#ifndef ROSAXIS_H
#define ROSAXIS_H

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
std_msgs::Empty outMsgEmpty;
std_msgs::Bool outMsgBool;
std_msgs::Int32 outMsg32;
std_msgs::Float32 outMsgFloat;
std_msgs::String outMsgString;
class RosSystem{
private:
	ros::Subscribe sub_sys_error;
	ros::Subscribe sub_cxt_switch;
    ros::Publisher pub_sys_error;
    ros::Publisher pub_cxt_switch;
public:
    RosAxis(uint16_t nodeID) :
		nodeID_(nodeID),
		pub_error_msg(assembleName("odrv/rtos", nodeID, "/error_msg"), &outMsgString),
		pub_sys_error(assembleName("odrv/rtos", nodeID, "/error"), &outMsg32),
		pub_cxt_switch(assembleName("odrv/rtos", nodeID, "/ctx_switch"), &outMsg32)
	{};
	void publishSysErrorMsg() {
		outMsgString.data = ch.dbg.panic_msg;
		pub_error_msg.publish(&outMsgString);
	}
	void publishSysError(){
		outMsg32.data = ch.dbg.isr_cnt;
		pub_sys_error.publish(&outMsg32);
	}
	void publishCxtSwitch(){
		outMsg32.data = ch.kernel_stats.n_ctxswc;
		pub_cxt_switch.publish(&outMsg32);
	}

	void publish(){
		publishSysErrorMsg();
		publishSysError();
		publishCxtSwitch();
	}

};