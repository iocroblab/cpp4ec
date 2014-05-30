/**
 * The drive moves with a TARGETVELOCITY during a preset TIME
 */

#include "EcMaster.h"
#include "EcSlaveTrack.h"
#include "EcSlave.h"

#include <iostream>
#include <vector>

#define TIME 3000000 //in us, actual is 3 seconds
#define MAXVELOCITY 250
#define MINVELOCITY -250

int main ()
{
  cpp4ec::EcMaster master("rteth1",1000000,true);	// 1ms and with DC
  std::vector<cpp4ec::EcSlave*> drivers;
  int status,position,velocity;
   
  try
  {
    drivers.resize(0);
    /* The master and slaves are configured  */
    master.preconfigure();
    /* The slaves vector is get, in this demo, one EcSlave is conected in the net */
    drivers = master.getSlaves();
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> DefaultParameters();
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> modeSetUp(VELOCITY_CONTROL,0);
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> TelegramType(VZ5);


    /* The master and slaves are configured  */
    master.configure();
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> ClearError(); //Clear error to clear the error F2174 Motor Encoder Loss
    /* The master and slaves are started  */
    usleep(250000); //Little waiting to avoid the error F2025
    master.start();


    std::cout<<"Start Reading....... "<<std::endl;
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(FIRST_ENTRY,status);
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(SECOND_ENTRY,position);
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(THIRD_ENTRY,velocity);
    std::cout<<"Status: "<<status<<" position "<<position<<" velocity "<<velocity<<std::endl;

    while (velocity != 250 )
    {
        std::cout<<"Enter a Velocity between "<<MAXVELOCITY<<"mm/s and "<<MINVELOCITY<<"mm/s or 0 to exit "<<std::endl;
        std::cin>>velocity;

        if((velocity < MAXVELOCITY) && (velocity > MINVELOCITY))
        {
            velocity = velocity*60*1000;
            ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(THIRD_ENTRY,velocity);
            master.update();
            usleep(TIME);
            for (int i=1;i<20;i++)
            {
                velocity = velocity/i;
                ((cpp4ec::EcSlaveTrack*) drivers[0]) -> writeMDT(THIRD_ENTRY,velocity);
                master.update();

            }
        }
     }



    std::cout<<"Finish Reading....... "<<std::endl;
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(FIRST_ENTRY,status);
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(SECOND_ENTRY,position);
    ((cpp4ec::EcSlaveTrack*) drivers[0]) -> readAT(THIRD_ENTRY,velocity);
    std::cout<<"Status: "<<status<<" position "<<position<<" velocity "<<velocity<<std::endl;



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
                                    
