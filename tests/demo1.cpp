/* 
 * 
 * Example code for EcMaster object
 *
 * Usage : demo1 
 *
 * This demo shows how to run an EcMaster object. The master prints on the screen
 * the different steps that does: configure the network and slaves, and reach the operational state.
 * 
 */

#include "EcMaster.h"
#include <iostream>

int main ()
{
  /*Intancied the EcMaster with a pediod of PDO comunication of 1 ms, without using Distributed Clocks and 
   *printing the slave information in the SlaveInfo.txt*/
  cpp4ec::EcMaster master("rteth0",1000000,false,true);
  
  try
  {
    master.preconfigure();
        
    /* The master and slaves are configured  */
    master.configure();
    /* The master and slaves are started  */  
    master.start();
    
    /*The demo waits 3 second but the master is sending PDO because the cyclic communication 
     *is already stablished (whithout new information)*/
    usleep(3000000);
    
    /* The master and slaves are stopped and reset */
    master.stop();
    master.reset();
  }
  catch (EcError& e)
  {
    /* if there are any expected exception it will be printed*/
    std::cout<<e.what()<<std::endl;
  }
    
  
  return (0);
}
