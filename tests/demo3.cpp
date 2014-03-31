/* 
 * Example code for writing velocities on SGDV servos
 *
 * Usage : demo3 
 *
 * This demo works setting the operational mode of the SGDV servos to 1. The demo set the position 
 * an the control word of how to move it. In the demo the servos move 90 degrees in relative mode
 * an then return to the initial position, setting 0 degrees in absolute mode.
 * 
 */

#include "EcMaster.h"
#include "EcSlaveSGDV.h"
#include "EcSlave.h"

#include <iostream>
#include <vector>

int main ()
{
  /* Intancied the defauld EcMaster: with a pediod of PDO comunication of 1 ms, without using Distributed Clocks and 
   * without slave information*/
  cpp4ec::EcMaster master;	
  std::vector<cpp4ec::EcSlave*> drivers;
  int32_t position;
     
  try
  {
    drivers.resize(0);
    master.configure();
    drivers = master.getSlaves();
  
    master.start(); 
    position = 90000;
    
    /* We set the desired position in each slave */
    for (int j = 0; j < drivers.size(); j++)
	((cpp4ec::EcSlaveSGDV*) drivers[j]) -> writePosition (position);
    
    /* The PDO outputs are updated, so they are sent to the RT thread. The velocities are sent to the slaves */
    master.update();    

    /* Send the desired ControlWord to start the next positioning, the servos starts positioning when the bit 4 
     * of the control word change from 0 to 1. The specified position is considered a relative position */
    for (int j = 0; j < drivers.size(); j++)
        ((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writeControlWord (CW_START_REL_POSITIONING);
    master.update();
    
    /* Set the bit 4 to 0 to be able to start a new positioning */    
    for (int j = 0; j < drivers.size(); j++)
        ((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writeControlWord (CW_START_REL_POSITIONING-16);
    master.update();    
 
    usleep(3000000);
    
    /* Return to the initial state using absolute position and controlWord */
    position = 0;

    for (int j = 0; j < drivers.size(); j++)
	((cpp4ec::EcSlaveSGDV*) drivers[j]) -> writePosition (position);
    master.update();    

    /* The specified position is considered an absolute position */
    for (int j = 0; j < drivers.size(); j++)
        ((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writeControlWord (CW_START_ABS_POSITIONING);
    master.update();
    
    /* Set the bit 4 to 0 to be able to start a new positioning */    
    for (int j = 0; j < drivers.size(); j++)
        ((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writeControlWord (CW_START_ABS_POSITIONING-16);
    master.update();    
 
    usleep(3000000);
    
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
                                    
