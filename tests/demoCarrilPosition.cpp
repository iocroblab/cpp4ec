/**
  * The drive goes from de actual position to the "MAXPOSITION" and the "MINPOSITION",
  * the MAX-MIN position cycle is repeated "CYCLECOUNT" times
  * All the process is done at a constant "VELOCITY"
  */

#include "EcMaster.h"
#include "EcSlaveTrack.h"
#include "EcSlave.h"


#include <iostream>
#include <vector>

#define MAXPOSITION 1300 //mm
#define MINPOSITION  600 //mm
#define CYCLECOUNT 1
#define VELOCITY 300 //mm/s

using namespace std;
cpp4ec::EcMaster master("rteth1",1000000,true);	// 1ms and with DC
std::vector<cpp4ec::EcSlave*> drivers;


int main ()
{

    int position, initialPosition;

    try
    {
      drivers.resize(0);
      master.preconfigure();
      /* The slaves vector is get, in this demo, one EcSlave is conected in the net */
      drivers = master.getSlaves();
      cout<<"Slave got"<<drivers.size()<<endl;
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> DefaultParameters();
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> modeSetUp(DRIVEINTERNALINTERPOLATION_CONTROL,0);
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> TelegramType(VZ7);
      cout<<"slecting modes"<<drivers.size()<<endl;

      //Configuration of the MDT and AT
      std::vector<int> idnlistMDT={258,259,260};
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> setMDT(idnlistMDT);
      std::vector<int> idnlistAT={51,40,437};
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> setAT(idnlistAT);
      cout<<"p2 configuration !!"<<endl;

      /* The master and slaves are configured  */
      master.configure();
      //((cpp4ec::EcSlaveTrack*) drivers[0]) -> ClearError(); //Clear error to clear the error F2174 Motor Encoder Loss
      /* The master and slaves are started  */
      usleep(250000); //Little waiting to avoid the error F2025
      master.start();

      int targetPosition;
      int cycle = 1;
      bool finish = false;
      int add=0;
      while (cycle<=CYCLECOUNT)
      {

              targetPosition=MAXPOSITION*10000;
              ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(SECOND_ENTRY,targetPosition);
              ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(THIRD_ENTRY,(VELOCITY+100*(add))*60*1000);
              add++;

              master.update();
              std::cout<<"sending "<<targetPosition<<std::endl;

              finish = false;
              while (!finish)
              {
                  ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(SECOND_ENTRY,position);
                  if(abs(position - targetPosition) < 10000 )
                      finish = true;
                  //if (position==0)
                  //    std::cout<<"The position is 0, ERROR!!!!!"<<std::endl;
                  usleep(1000);
              }

              targetPosition=MINPOSITION*10000;
              ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(SECOND_ENTRY,targetPosition);
              ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(THIRD_ENTRY,(VELOCITY+100*(add))*60*1000);
              add++;
              master.update();
              std::cout<<"sending "<<targetPosition<<std::endl;

              finish = false;
              while (!finish)
              {
                  ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(SECOND_ENTRY,position);
                  if(abs(position - targetPosition) < 10000 )
                      finish = true;
                  if (position==0)
                      std::cout<<"The position is 0, ERROR!!!!!"<<std::endl;
                  usleep(1000);
              }
              cycle ++;
          }
      usleep(1000000);
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
                                    
