/* 
 * Example code for writing velocities on SGDV servos
 *
 * Usage : demo2 
 *
 * This demo is designed for three SGDV servos. The demo set one positive velocity in one direction
 * for 5 seconds and then the opposit velocity during 5 seconds more.
 * 
 */

#include "EcMaster.h"
#include "EcSlaveSGDV.h"
#include "EcSlave.h"

#include <iostream>
#include <vector>

int main ()
{
  /*Intancied the defauld EcMaster: with a pediod of PDO comunication of 1 ms, without using Distributed Clocks and 
   without slave information*/
  cpp4ec::EcMaster master;	
  std::vector<cpp4ec::EcSlave*> drivers;
  int32_t velocity;
   
  try
  {
    drivers.resize(0);
    /* The master and slaves are configured  */
    master.configure();
    /* The slaves vector is get, in this demo, three EcSlaveSGDV are conected in the net */
    drivers = master.getSlaves();
  
    master.start(); 
    
    velocity = 50000;
    for (int j = 0; j < drivers.size(); j++)
    {
	/* We set the desired velocity in each slave */
        ((cpp4ec::EcSlaveSGDV*) drivers[j]) -> writeVelocity (velocity);
    }
    /* The PDO outputs are updated, so they are sent to the RT thread. The velocities are sent to the slaves */
    master.update();    
    usleep(5000000); 
    
    velocity = -50000;
    for (int j = 0; j < drivers.size(); j++)
    {
        ((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writeVelocity (velocity);
    }     
    master.update();    
    usleep(5000000); 
    
    /* The master and slaves are stopped and reset */
    master.stop();
    master.reset();    
     
  }
  catch (EcError& e)
  {
    /* if there are any expected exception it will be printed on the screen */ 
    std::cout<<e.what()<<std::endl;  
  }
  
  return (0);
}
                                    
