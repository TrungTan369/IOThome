#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include <Arduino.h>

class task{
public:
      void (*pTask) (void);
      unsigned int Delay;
      unsigned int Period;
      task * next;
      task * prev;
public:
      task(void (*pTask) (void), unsigned int Delay, unsigned int Period);
};

class ListTask{
private:
      task * head;
      task * tail;
      unsigned int numTask;
public:
      ListTask();
      void run();
      void SCH_Add_Task(void (*pTask) (void), unsigned int Delay, unsigned int Period);
      int SCH_Delete_Task(void (*)());
      void SCH_Dispatch_Task();
};


#endif /* INC_SCHEDULER_H_ */