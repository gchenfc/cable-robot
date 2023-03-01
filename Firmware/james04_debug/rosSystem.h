#ifndef ROSYSTEM_H
#define ROSYSTEM_H
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>


class RosSystem{
private:

	std_msgs::Bool outMsgBool;
    std_msgs::Int32 outMsg32;
    std_msgs::Float32 outMsgFloat;
    std_msgs::String outMsgString;
	//ros::Subscriber sub_sys_error;
	//ros::Subscriber sub_cxt_switch;
	ros::Publisher pub_error_msg;
    ros::Publisher pub_sys_error;
    ros::Publisher pub_cxt_switch;
    
public:
    RosSystem() :
		pub_error_msg("odrv/rtos/error_msg", &outMsgString),
		pub_sys_error("odrv/rtos/sys_error", &outMsg32),
		pub_cxt_switch("odrv/rtos/ctx_switch", &outMsg32)
	{
		nh.advertise(pub_error_msg);
		nh.advertise(pub_sys_error);
		nh.advertise(pub_cxt_switch);
	};
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

#endif