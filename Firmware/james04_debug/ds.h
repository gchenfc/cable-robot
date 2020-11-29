#include "ChRt.h"
#include <FlexCAN.h>
struct Queue_Node{
	void* item = NULL;
	void* next = NULL;
};
class Queue{
public:
	ch_mutex lock;
	Queue_Node* front = NULL;
	Queue_Node* end = NULL;

};
struct mem{
	uint16_t id; 
	uint64_t val;
};

struct can_msg {
	CAN_message_t msg;
	int axis;
	int status = 0; // 0 -> unusable 1 -> usable
};

