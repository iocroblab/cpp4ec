#ifndef SERVOS_RT_H
#define SERVOS_RT_H

extern "C"
{
#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>
}

namespace servos
{

   static void ethercatLoop(void *unused);
    
   static RT_MUTEX mutex;
   static RT_TASK task;
   static RT_TASK program;
      
   ///realtime functions   
   inline void ethercatLoop(void *unused)
    {
      int nRet;
      while(1)
      {
         rt_mutex_acquire(&mutex, TM_INFINITE);
         nRet=ec_send_processdata();
         nRet=ec_receive_processdata(EC_TIMEOUTRET);
         rt_mutex_release(&mutex);
         rt_task_wait_period(NULL);
      }      
    }
};
#endif //SERVOS_RT_H