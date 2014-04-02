/* 
 * Example code for using general function
 *
 * Usage : demo4  
 *	   - In the configure_SGDV_n.xml must be selected operational mode 3. If it is chossed another one
 *	     might do strange things even with the cyclic velocity mode = 9.
 *	   - In the profile velcity mode parameters like profile acceleration or deceleration can be changed
 *	     to fit user necessities.
 *	   - In the profile position mode parameters like profile velocity, acceleration or deceleration can be changed
 *	     to fit user necessities. 
 *		  
 *
 * The demo move the motors by velocity (Opmode = 3) and then change to position (OpMode = 1). The demo 
 * set a velocity for the motors for a while and then stop them. When the movement
 * has been finished, the actual position is readed for each slave. After that, the operational mode
 * is switched to position and then returned the servos to the inital position using the encoders information.
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
   * without slave information
   */
  cpp4ec::EcMaster master;	
  std::vector<cpp4ec::EcSlave*> drivers;
  int32_t velocity;
  int32_t position;
   
  try
  {
    drivers.resize(0);
    /* The master and slaves are configured  */
    master.configure();
    /* The slaves vector is get, in this demo, three EcSlaveSGDV are conected in the net */
    drivers = master.getSlaves();
  
    master.start(); 
    velocity = 100000;
    
    /* We set the desired velocity using the generic funtion to write in the receive PDO.
     * In the PDO mapping the velocity is set in the third entry. 
     */
    for (int j = 0; j < drivers.size(); j++)
	((cpp4ec::EcSlaveSGDV*) drivers[j]) -> writePDO (cpp4ec::EcSlaveSGDV::THIRD_ENTRY,velocity);
    
    /* The PDO outputs are updated, so they are sent to the RT thread. The velocities are sent to the slaves */
    master.update();    
    usleep(3000000);
    
    /* Stops the platform */    
    velocity = 0;
    for (int j = 0; j < drivers.size(); j++)
        ((cpp4ec::EcSlaveSGDV*) drivers[j]) -> writePDO (cpp4ec::EcSlaveSGDV::THIRD_ENTRY,velocity);
    
    /* The PDO outputs are updated, so they are sent to the RT thread. The velocities are sent to the slaves */
    master.update();    
    usleep(1000000);
    
    int8_t mode = 1;
    /* Change the Operation mode to 1 (profile position mode)*/
    for (int j = 0; j < drivers.size(); j++)
	((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writePDO (cpp4ec::EcSlaveSGDV::FIFTH_ENTRY, mode);

    std::cout<<"The slaves' position are: ";
    for (int j = 0; j < drivers.size(); j++)
    {
	/*Read the position actual value that is in the second entry of the transmit PDO */
	((cpp4ec::EcSlaveSGDV*)drivers[j]) -> readPDO (cpp4ec::EcSlaveSGDV::SECOND_ENTRY, position);
	/* We set this position in the opposit sense to return to the initial position
	 * This demo works in realtive reference
	 */
	((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writePDO(cpp4ec::EcSlaveSGDV::SECOND_ENTRY,-position);
        std::cout<<(int)position<<" ";
    }
    std::cout<<" millidegrees"<<std::endl;

    master.update();    
    usleep(1000); 
    
    /* Send the desired ControlWord to start moving*/
    for (int j = 0; j < drivers.size(); j++)
        ((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writePDO(cpp4ec::EcSlaveSGDV::FIRST_ENTRY,cpp4ec::EcSlaveSGDV::CW_START_REL_POSITIONING);
  
    master.update(); 
    for (int j = 0; j < drivers.size(); j++)
        ((cpp4ec::EcSlaveSGDV*)drivers[j]) -> writePDO(cpp4ec::EcSlaveSGDV::FIRST_ENTRY,cpp4ec::EcSlaveSGDV::CW_START_REL_POSITIONING-16);
  
    master.update();
    usleep(5000000);
    
    std::cout<<"The final slaves' position are: ";
    for (int j = 0; j < drivers.size(); j++)
    {
	/*Read the position actual value that is in the second entry of the transmit PDO */
	((cpp4ec::EcSlaveSGDV*)drivers[j]) -> readPDO (cpp4ec::EcSlaveSGDV::SECOND_ENTRY, position);
	std::cout<<(int)position<<" ";
    }
    std::cout<<" millidegrees"<<std::endl;
    
    
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
                                    
