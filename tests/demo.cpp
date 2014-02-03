#include "EcMaster.h"

#include <iostream>

int main ()
{
  cpp4ec::EcMaster master;
  try
  {
    std::cout<<"trying"<<std::endl;
    master.preconfigure();
    master.configure();
    master.start();
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