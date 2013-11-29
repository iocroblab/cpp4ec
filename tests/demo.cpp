#include "EcMaster.h"

int main ()
{
  ec4cpp::EcMaster master;
  
  master.preconfigure();
  master.configure();
  master.reset();
  
  return (0);
}