/**
  * The drive goes to a position introduced with the keyboard,
  * the target position has to stay between the "MAXPOSITION" and "MINPOSITION"
  * If the input with the keyboard is 0 the program ends
  * All the process is made at preset VELOCITY
  * The mode that has been used is position control, so the position is made with a interval incrementation of the position
  * the end of the trajectory is not really smoothed, I need to improve the demo
  */
#include "EcMaster.h"
#include "EcSlaveTrack.h"
#include "EcSlave.h"



#include <iostream>
#include <vector>

#define MAXPOSITION 1800
#define MINPOSITION  100
#define VELOCITY 300 //mm/s


cpp4ec::EcMaster master("rteth1",1000000,true);	// 1ms and with DC
std::vector<cpp4ec::EcSlave*> drivers;


int main ()
{

    int position, initialPosition;

    try
    {
      drivers.resize(0);
      master.preconfigure();
      /* The slaves vector is get, in this demo, three EcSlaveSGDV are conected in the net */
      drivers = master.getSlaves();
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> DefaultParameters();
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> modeSetUp(DRIVEINTERNALINTERPOLATION_CONTROL,0);
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> TelegramType(VZ7);
      //Configuration of the MDT and AT
      std::vector<int> idnlistMDT={258,259,260};
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> setMDT(idnlistMDT);
      std::vector<int> idnlistAT={51,40,437};
      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> setAT(idnlistAT);
      /* The master and slaves are configured  */
      master.configure();
      //((cpp4ec::EcSlaveTrack*) drivers[0]) -> ClearError(); //Clear error to clear the error F2174 Motor Encoder Loss
      /* The master and slaves are started  */
      usleep(250000); //Little waiting to avoid the error F2025
      master.start();




      int targetPosition=1000;
      bool finish = false;
      int sw, vel, acce;
      while (targetPosition != 0 )
      {
          std::cout<<"Enter a position between "<<MAXPOSITION<<"mm and "<<MINPOSITION<<"mm or 0 to exit "<<std::endl;
          std::cin>>targetPosition;

          if((targetPosition < MAXPOSITION) && (targetPosition > MINPOSITION))
          {
              targetPosition = targetPosition*10000;


              ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(SECOND_ENTRY,targetPosition);
              ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(THIRD_ENTRY,VELOCITY*60*1000);
              master.update();
              finish = false;
              while (!finish)
              {
                  ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(SECOND_ENTRY,position);
                  ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(FIRST_ENTRY,sw);
                  ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(THIRD_ENTRY,vel);
                  ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(FOURTH_ENTRY,acce);


                  if(abs(position - targetPosition) < 10000 )
                      finish = true;
                  if(position ==0 )
                  {
                      std::cout<<"position is 0"<<std::endl;
                      std::cout<<"sw is "<<sw<<std::endl;
                      std::cout<<"velocity is "<<vel<<std::endl;
                      std::cout<<"new sw is "<<acce<<std::endl;

                      acce=286331153;
                      ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(FOURTH_ENTRY,acce);
                      master.update();

                      //return(1);
                  }
                  usleep(1000);

              }
          }
      }
      //usleep(1000000);
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
                                    
