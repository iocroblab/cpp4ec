#include "EcMaster.h"

int main ()
{
  cpp4ec::EcMaster master;
  
  master.preconfigure();
  master.configure();
  master.reset();
  
  return (0);
}