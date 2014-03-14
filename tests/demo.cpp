#include "EcMaster.h"

#include <iostream>

int main ()
{
  cpp4ec::EcMaster master;
  try
  {
    master.configure();
    master.start();
    usleep(3000000);
    master.stop();
    master.reset();
  }
  catch (EcError& e)
  {
    std::cout<<e.what()<<std::endl;
  }
    
  
  return (0);
}