#include "scheduler.h"


task:: task(void (*pTask) (void), unsigned int Delay, unsigned int Period) :
            pTask(pTask), Delay(Delay), Period(Period), next(nullptr), prev(nullptr) {}

ListTask:: ListTask() : head(nullptr), tail(nullptr), numTask(0) {}

void ListTask:: run(){
      if (head != nullptr && head->Delay > 0) {
            head->Delay--;
        }
}

void ListTask:: SCH_Add_Task(void (*pTask) (void), unsigned int Delay, unsigned int Period){
      task * nTask = new task(pTask, Delay/10, Period/10); 
      if(head == nullptr){
            head = nTask;
            tail = nTask;
      }
      else{
            task * cur = head;
            while(cur != nullptr && nTask->Delay >= cur->Delay){
                  nTask->Delay = nTask->Delay - cur->Delay;
                  cur = cur->next;
            }
            if(cur == head){
                  nTask->next = head;
                  head->prev = nTask;
                  head->Delay = head->Delay - nTask->Delay;
                  head = nTask;
            }
            else if(cur == nullptr){
                  nTask->prev = tail;
                  tail->next = nTask;
                  tail = nTask;
            }
            else{
                  nTask->next = cur;
                  nTask->prev = cur->prev;
                  cur->prev->next = nTask;
                  cur->prev = nTask;
                  cur->Delay = cur->Delay - nTask->Delay;
            }
      }
      numTask++;
}

void ListTask:: SCH_Dispatch_Task(){
      if(this->head == nullptr){
            return;
      }
      if(head->Delay <= 0 ){
            head->pTask();
            task * temp = head;
            head = head->next;
            if(temp->Period > 0){
                  SCH_Add_Task(temp->pTask, temp->Period * 10, temp->Period * 10);
            }
            numTask--;
            delete temp;
      }
}

int ListTask:: SCH_Delete_Task(void (*function)()){
	if(numTask == 0)
		return 0;
	if(numTask == 1){
		task * temp = head;
		head = tail = nullptr;
		numTask = 0;
		delete temp;
		return 1;
	}
	task * temp = head;
	while(temp != nullptr){
		if(temp->pTask == function){
			if(temp == head){ // Delete head
				temp->next->prev = nullptr;
				head = temp->next;
				head->Delay = head->Delay + temp->Delay;
			}
			else if (temp == tail){ // Delete tail
				temp->prev->next = nullptr;
				tail = temp->prev;
			}
			else{
				temp->prev->next = temp->next;
				temp->next->prev = temp->prev;
				temp->next->Delay = temp->next->Delay + temp->Delay;
			}
			numTask--;
			delete temp;
			return 1;
		}
		temp = temp->next;
	}
	return 0;
}