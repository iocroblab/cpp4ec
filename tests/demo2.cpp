#include "EcMaster.h"
#include "EcSlaveSGDV.h"
#include "EcSlave.h"

#include <iostream>
#include <vector>

int main ()
{
  cpp4ec::EcMaster master(1000000,true);// 1ms using DC
  std::vector<cpp4ec::EcSlave*> drivers;
  int32_t velocity;
   
  try
  {
    master.configure();
    drivers=master.getSlaves();
    
    master.start(); 
    velocity = 50000;
    for (int j = 0; j<3; j++)
    {
        ((cpp4ec::EcSlaveSGDV*)drivers[j])->writeVelocity (velocity);
        usleep(1000);
        master.update();
    }    
    usleep(5000000);    
    master.stop();
    
    usleep(1000000);
    
    velocity = -50000;
    master.start();
    for (int j = 0; j<3; j++)
    {
        ((cpp4ec::EcSlaveSGDV*)drivers[j])->writeVelocity (velocity);
        usleep(1000);
        master.update();
    }
    usleep(5000000);    
    master.stop();                                
        
          
    master.reset(); 
  }
  catch (EcError& e)
  {
    std::cout<<e.what()<<std::endl;  
  }
  
  return (0);
}
                                    
