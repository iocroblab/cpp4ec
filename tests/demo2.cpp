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
    master.preconfigure();
    master.configure();
    master.start(); 
    drivers=master.getSlaves();
    int32_t velocity;
    for (int j = 0; j<3; j++)
    {
      ((cpp4ec::EcSlaveSGDV*)drivers[j])->readVelocity (velocity);
  //     master.update_ec();
       std::cout<<"velocity "<<velocity<<std::endl;
    }
                         
    
    velocity = 50000;
 //   for(int k=0;k<1;k++)
 //   {
    for (int j = 0; j<3; j++)
    {
        ((cpp4ec::EcSlaveSGDV*)drivers[j])->writeVelocity (velocity);
        usleep(1000);
        master.update();
    }
                                                      
    usleep(100000);
    
   // }
    
    usleep(5000000);
    master.stop();  
    master.reset(); 
  }
  catch (EcError& e)
  {
    std::cout<<"Exeption"<<std::endl;
    std::cout<<e.what()<<std::endl;  
  }
  
  return (0);
}
                                    
