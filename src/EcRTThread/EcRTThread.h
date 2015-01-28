#ifndef _realtimetask_
#define _realtimetask_

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>

#define XDDP_PORT_INPUT "EcMaster-xddp-input"
#define XDDP_PORT_OUTPUT "EcMaster-xddp-output"

int taskFinished;
void rt_thread(void *argument);



#endif
