#include <FlexCAN.h>

void msgCopy(CAN_message_t& origin, CAN_message_t* copy){
	copy -> id = origin.id;
	copy -> ext = origin.ext;
	copy -> rtr = origin.rtr;
	copy -> len = origin.len;
	copy -> timeout = origin.timeout;
	memcpy(&(copy->buf),&(origin.buf),8);
};
