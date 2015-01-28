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
#include <rtdm/rtipc.h>
#include <native/timer.h>

#define timestampSize 8

void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime);
int64 integral = 0;

void rt_thread(void *unused)
{
   mlockall(MCL_CURRENT | MCL_FUTURE);

   struct rtipc_port_label plabel_in, plabel_out;
   struct sockaddr_ipc saddr_in, saddr_out;

   int64 timestamp=0, lastdctime=0, dctime=0,toff = 0, cycletime = 1000000;
   int ret_in, ret_out, s_input, s_output, nRet;
   int inputSize = 0, outputSize = 0;
   int i;

   //The input buffer have added space for include timestamp (the DCtime is a 64bits clock)
   inputSize = ec_slave[0].Ibytes + ec_slavecount * timestampSize;
   outputSize = ec_slave[0].Obytes;

   char * rtinputbuf = (char*) malloc(inputSize*(sizeof(char)));
   char * rtoutputbuf = (char*) malloc(outputSize*(sizeof(char)));
   memset(rtoutputbuf,0, outputSize);
   memset(rtinputbuf,0, inputSize);

   /*need to put a lot of stuff to create
    * two sockets: one to receive data from nonrt part
    * and another to send data to the nonrt part
    * use same option as the xenomai xddp-label example
    *
    * the output part came from the NRT part to the RT world
    * the input part came from the RT part to the NRT world
    *
    *
    * Get a datagram socket to bind to the RT endpoint. Each
    * endpoint is represented by a port number within the XDDP
    * protocol namespace.
    **/
    s_output = rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
    s_input =  rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);

    if (s_output < 0)
      perror("socket");

    if (s_input < 0)
      perror("socket");

    /*
     * Set a port label. This name will be registered when
     * binding, in addition to the port number (if given).
     */
    strcpy(plabel_out.label, XDDP_PORT_OUTPUT);
    ret_out = rt_dev_setsockopt(s_output, SOL_XDDP, XDDP_LABEL, &plabel_out, sizeof(plabel_out));
    if (ret_out)
        perror("setsockopt");

    /*
     * Set a port label. This name will be used to find the peer
     * when connecting, instead of the port number.
     */
    strcpy(plabel_in.label, XDDP_PORT_INPUT);
    ret_in = rt_dev_setsockopt(s_input, SOL_XDDP, XDDP_LABEL,&plabel_in, sizeof(plabel_in));
    if (ret_in)    
      perror("setsockopt");

    /*
     * Bind the socket to the port, to setup a proxy to channel
     * traffic to/from the Linux domain. Assign that port a label,
     * so that peers may use a descriptive information to locate
     * it. For instance, the pseudo-device matching our RT
     * endpoint will appear as
     * /proc/xenomai/registry/rtipc/xddp/<XDDP_PORT_LABEL> in the
     * Linux domain, once the socket is bound.
     *
     * saddr.sipc_port specifies the port number to use. If -1 is
     * passed, the XDDP driver will auto-select an idle port.
     */

    /* Output socket from NRT to RT */
    memset(&saddr_out, 0, sizeof(saddr_out));
    saddr_out.sipc_family = AF_RTIPC;
    saddr_out.sipc_port = -1;
    ret_out = rt_dev_bind(s_output, (struct sockaddr *)&saddr_out, sizeof(saddr_out));
    if (ret_out)
      perror("bind");


    /*Input socket from RT to NRT*/
    memset(&saddr_in, 0, sizeof(saddr_in));
    saddr_in.sipc_family = AF_RTIPC;
    saddr_in.sipc_port = -1; /* Tell XDDP to search by label. */
    ret_in = rt_dev_bind(s_input, (struct sockaddr *)&saddr_in, sizeof(saddr_in));
    if (ret_in)
        perror("bind");

    RTIME date= rt_timer_read();
    while(!taskFinished)
    {
       //Calculate next time execution
       date = date + cycletime + toff;
       //Sleep until time is reached
       rt_task_sleep_until (date);
       date = rt_timer_read();

       /* Get packets from NRT to RT */
       ret_out = rt_dev_recvfrom(s_output, rtoutputbuf, outputSize, MSG_DONTWAIT, NULL, 0);
       if (ret_out <= 0)
       {
           //perror("recvfrom");//nothing to do, no new data transferred
       }else{
           //received some data from the buffer
           memcpy (ec_slave[0].outputs, rtoutputbuf , ec_slave[0].Obytes);
       }

       nRet=ec_send_processdata();
       if(nRet == 0)
           rt_printf("Send failed");

       nRet=ec_receive_processdata(EC_TIMEOUTRET);
       if(nRet == 0)
           rt_printf("Recieve failed");

       //Convert 32 bit clock of Distributed clock to 64 bit
       dctime = ec_DCtime;
       if(dctime >= lastdctime)
           timestamp = (timestamp & 0xFFFFFFFF00000000) + dctime;
       else
           timestamp = (timestamp & 0xFFFFFFFF00000000) + 0x0100000000 + dctime;
       lastdctime = dctime;

       int offSet = 0;
       for (i = 1; i <= ec_slavecount; i++)
       {
         memcpy (rtinputbuf + offSet ,ec_slave[i].inputs, ec_slave[i].Ibytes);
         memcpy (rtinputbuf + offSet + ec_slave[i].Ibytes, &timestamp, timestampSize);
         offSet = offSet + ec_slave[i].Ibytes  + timestampSize;
       }

       //Send packets from RT to NRT
       ret_in = rt_dev_sendto(s_input, rtinputbuf, inputSize, 0, NULL, 0);
       if(ret_in != inputSize)
           perror("sendto");

       //Caculate the offset to get the task and Distributed Clock synced
       ec_sync(timestamp,cycletime,&toff);


    }
    //Free memory buffer
    free(rtinputbuf);
    free(rtoutputbuf);
    //Close sockets
    rt_dev_close(s_output);
    rt_dev_close(s_input);
     
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
