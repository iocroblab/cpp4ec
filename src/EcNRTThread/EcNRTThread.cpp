#include "EcNRTThread.h"
#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
#include <soem/osal.h>
#include <soem/oshw.h>
//#include <sys/mman.h>
//#include <getopt.h>
//#include <execinfo.h>
//#include <unistd.h>
//#include <getopt.h>
//#include <execinfo.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define timestampSize 8
int NRTtaskFinished = false;
void nrt_thread(void *unused)
{
    int nRet;
    boost::asio::io_service io;
    boost::asio::deadline_timer timer(io);

    int period = 1000000;
    timer.expires_from_now(boost::posix_time::milliseconds(period));
    while(!NRTtaskFinished)
    {
        nRet=ec_send_processdata();
        if(nRet == 0)
            printf("Send failed");

        nRet=ec_receive_processdata(EC_TIMEOUTRET);
        if(nRet == 0)
            printf("Recieve failed");
        timer.wait();
        timer.expires_from_now(boost::posix_time::milliseconds(period));

    }
}
