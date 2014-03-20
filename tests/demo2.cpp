#include "EcMaster.h"
#include "EcSlaveSGDV.h"
#include "EcSlave.h"

#include <iostream>
#include <vector>

int main ()
{
  cpp4ec::EcMaster master;	// 1ms and without DC, by default
  std::vector<cpp4ec::EcSlave*> drivers;
  int32_t velocity;
   
  try
  {
    drivers.resize(0);
    /* The master and slaves are configured  */
    master.configure();
    /* The slaves vector is get, in this demo, three EcSlaveSGDV are conected in the net */
    drivers = master.getSlaves();
    /* The master and slaves are started  */    
    master.start(); 
    
    velocity = 50000;
    for (int j = 0; j < drivers.size(); j++)
    {
	/* We set the desired velocity in each one */
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
    /* if there are any spected error it would be printed on the screen */ 
    std::cout<<e.what()<<std::endl;  
  }
  
  return (0);
}
                                    
