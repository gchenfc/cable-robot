#include "ChRt.h"
struct Queue_Node{
	void* item;
	void* next;
};
class Queue{
public:
	ch_mutex lock;
	Queue_Node* front;
	Queue_Node* end;

};
