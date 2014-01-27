#ifndef REALTIMETASK_H
#define REALTIMETASK_H

extern "C"
{
#include <soem/ethercattype.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>
#include <soem/nicdrv.h>
}

#include <native/task.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <rtdk.h>
#include <rtdm/rtipc.h>
#include <signal.h>

#define XDDP_PORT_INPUT "EcMaster-xddp-input"
#define XDDP_PORT_OUTPUT "EcMaster-xddp-output"

namespace cpp4ec
{
   void realtime_thread(void *unused);
   RT_TASK task;  
   extern RT_MUTEX mutex;
   RT_MUTEX mutex;

   
inline void realtime_thread(void *unused)
{
   struct rtipc_port_label plabel_in, plabel_out;
   struct sockaddr_ipc saddr_in, saddr_out;
   
   struct timespec ts;
   struct timeval tv;
   socklen_t addrlen;
   
   int ret_in, ret_out, s_input, s_output, nRet;
   int inputSize = 0, outputSize = 0;
   
   for(int i = 1; i <= ec_slavecount; i++)
   {
      inputSize += ec_slave[i].Ibytes;
      outputSize += ec_slave[i].Obytes;
   }
   
   char * rtinputbuf = new char[inputSize];
   char * rtoutputbuf = new char[outputSize];
   
   
   /*need to put a lot of stuff to create
    * two sockets: one to receive data from nonrt part
    * and another to send data to the nonrt part
    * use same option as the xenomai xddp-label example
    * 
    * the output part came from the NRT part to the RT world
    * the input part came from the RT part to the NRT world
    *         
    *         /*
    * Get a datagram socket to bind to the RT endpoint. Each
    * endpoint is represented by a port number within the XDDP
    * protocol namespace.
    */
    s_output = socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
    s_input = socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
    
    
    if (s_output < 0) {
//        perror("socket");
//        exit(EXIT_FAILURE); //THROW EXCEPTION
    }
    if (s_input < 0) {
//        perror("socket");
//        exit(EXIT_FAILURE); //THROW EXCEPTION
    }
    
    
    /*
     * Set a port label. This name will be registered when
     * binding, in addition to the port number (if given).
     */
    strcpy(plabel_out.label, XDDP_PORT_OUTPUT);
    ret_out = setsockopt(s_output, SOL_XDDP, XDDP_LABEL, &plabel_out, sizeof(plabel_out));
    if (ret_out){}
//        fail("setsockopt, output");
    
    
    
    /*
     * Set the socket timeout; it will apply when attempting to
     * connect to a labeled port, and to recvfrom() calls. The
     * following setup tells the XDDP driver to wait for at most
     * one second until a socket is bound to a port using the same
     * label, or return with a timeout error.
     */
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    ret_in = setsockopt(s_input, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (ret_in){}
//        fail("setsockopt, input");
    /*
     * Set a port label. This name will be used to find the peer
     * when connecting, instead of the port number.
     */
    strcpy(plabel_in.label, XDDP_PORT_INPUT);
    ret_in = setsockopt(s_input, SOL_XDDP, XDDP_LABEL,&plabel_in, sizeof(plabel_in));
    if (ret_in){}
//        fail("setsockopt, input");
//     
    
    
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
    
    /*
     * Output!!!!!
     * from NRT to RT
     */
    memset(&saddr_out, 0, sizeof(saddr_out));
    saddr_out.sipc_family = AF_RTIPC;
    saddr_out.sipc_port = -1;
    ret_out = bind(s_output, (struct sockaddr *)&saddr_out, sizeof(saddr_out));
    if (ret_out){}
//        fail("bind");
    
    
    /*
     * Input!!!!!
     * from RT to NRT
     */
    
    memset(&saddr_in, 0, sizeof(saddr_in));
    saddr_in.sipc_family = AF_RTIPC;
    saddr_in.sipc_port = -1; /* Tell XDDP to search by label. */
    ret_in = connect(s_input, (struct sockaddr *)&saddr_in, sizeof(saddr_in));
    if (ret_in){}
//        fail("connect");
    /*
     * We succeeded in making the port our default destination
     * address by using its label, but we don't know its actual
     * port number yet. Use getpeername() to retrieve it.
     */
    addrlen = sizeof(saddr_in);
    ret_in = getpeername(s_input, (struct sockaddr *)&saddr_in, &addrlen);
    if (ret_in || addrlen != sizeof(saddr_in)){}
//        fail("getpeername, input");
    //rt_printf("%s: NRT peer is reading from /dev/rtp%d\n",          __FUNCTION__, saddr_in.sipc_port);
       
       
       for (;;) {
          /* Get packets relayed by the regular thread */
          ret_out = recvfrom(s_output, rtoutputbuf, outputSize, 0, NULL, 0);
          if (ret_out <= 0)
          {
             //nothing to do, no new data transferred
             //rt_printf("%s: \"%.*s\" relayed by peer\n", __function__, ret, buf);
          } 
          
          else{
             //received some data from the buffer 
             //traffic that data from
             //rtoutputbuf to ec_slave[i].output
             //copiar del outputbuf als ec_slave, tenint en compte el OBytes de cada slave
             int offSet = 0;
             for ( int i = 1; i<=ec_slavecount; i++)    
		{
		    memcpy (ec_slave[i].outputs, rtoutputbuf + offSet, ec_slave[i].Obytes);
		    offSet += ec_slave[i].Obytes;
		}
             
          }
          nRet=ec_send_processdata();
          //make some check of sending data
          nRet=ec_receive_processdata(EC_TIMEOUTRET);
          
          //now, we need to transmit the data outside
          //from ec_slave[i].inputs
          //build rtinputbuf
	    int offSet = 0;
	    for ( int i = 1; i<=ec_slavecount; i++)    
	    {
		memcpy (rtinputbuf + offSet ,ec_slave[i].inputs, ec_slave[i].Ibytes);
		offSet += ec_slave[i].Ibytes;
	    }
	  
	  
          ret_in = sendto(s_input, rtinputbuf, inputSize, 0, NULL, 0);
          
          rt_task_wait_period(NULL);
       }
       
}
    
};
#endif 