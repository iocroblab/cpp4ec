#ifndef _nrealtimetask_
#define _nrealtimetask_

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>


//int NRTtaskFinished;
void nrt_thread(int period);



#endif
