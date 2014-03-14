#include "EcMaster.h"
#include "EcSlaveSGDV.h"
#include "EcSlave.h"

#include <iostream>
#include <vector>

int main ()
{
  cpp4ec::EcMaster master;
  std::vector<cpp4ec::EcSlave*> drivers;
  try
  {
    master.configure();
    master.start(); 
    drivers=master.getSlaves();
    int32_t velocity;
    velocity = 50000;

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
                                    
