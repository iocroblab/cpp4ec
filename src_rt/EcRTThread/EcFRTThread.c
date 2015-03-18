#include "EcRTThread.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <soem/osal.h>
#include <soem/oshw.h>
#include <sys/mman.h>
#include <getopt.h>
#include <execinfo.h>
#include <unistd.h>
#include <getopt.h>
#include <execinfo.h>
//socket header
#include <native/timer.h>
#include <native/mutex.h>

#define timestampSize 8

void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime);
int64 integral = 0;

extern int64 EcTimeStamp;
int64 EcTimeStamp=0;

void rt_thread(void *argument)
{
   RT_MUTEX mutex;
   int cycletime = *((int64*)argument);

   int64 lastdctime=0, dctime=0,toff = 0;
   int nRet;
   rt_mutex_create (&mutex, "master_mutex");
   RTIME date= rt_timer_read();

   while(!taskFinished)
   {
       //Calculate next time execution
       date = date + cycletime + toff;
       //Sleep until time is reached
       rt_task_sleep_until (date);
       date = rt_timer_read();       
       rt_mutex_acquire (&mutex,TM_INFINITE);
       nRet=ec_send_processdata();
       if(nRet == 0)
           rt_printf("Send failed");

       nRet=ec_receive_processdata(EC_TIMEOUTRET);
       if(nRet == 0)
           rt_printf("Recieve failed");

       //Convert 32 bit clock of Distributed clock to 64 bit
       dctime = ec_DCtime;
       if(dctime >= lastdctime)
           EcTimeStamp = (EcTimeStamp & 0xFFFFFFFF00000000) + dctime;
       else
           EcTimeStamp = (EcTimeStamp & 0xFFFFFFFF00000000) + 0x0100000000 + dctime;
       lastdctime = dctime;
       rt_mutex_release (&mutex);

       //Caculate the offset to get the task and Distributed Clock synced
       ec_sync(EcTimeStamp,cycletime,&toff);
   }
   rt_mutex_delete (&mutex);
}

/* PI controller calculation to get linux time synced to DC time
 * The controller and parameters ara obtained from ebox.c example from SOEM repository
 **/
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   int64 delta;
   /* set linux sync point 50us before than DC sync, Named Master user shift time in documentation */
   delta = (reftime + 50000) % cycletime;
   if(delta> (cycletime /2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral /20);
}
