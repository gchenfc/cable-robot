#ifndef _ROS_robot_controllers_msgs_QueryControllerStatesActionResult_h
#define _ROS_robot_controllers_msgs_QueryControllerStatesActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "robot_controllers_msgs/QueryControllerStatesResult.h"

namespace robot_controllers_msgs
{

  class QueryControllerStatesActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef robot_controllers_msgs::QueryControllerStatesResult _result_type;
      _result_type result;

    QueryControllerStatesActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "robot_controllers_msgs/QueryControllerStatesActionResult"; };
    const char * getMD5(){ return "9a8e94275d921606c4552f40989a801e"; };

  };

}
#endif
