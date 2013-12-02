#include "EcMaster.h"

#include <sys/mman.h>
#include <iostream>
//#include "masterStructures.hpp"
#include <fstream> //Just to verify the time

//Xenomai
#include <native/task.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>

//Thread loop
#include "servos_rt.h"
#include "EcSlaveSGDV.h"



namespace ec4cpp
{

EcMaster::EcMaster(int cycleTime) : ethPort ("rteth0"), m_cycleTime(cycleTime)
{
   //reset del iomap memory
   for (size_t i = 0; i < 4096; i++)
      m_IOmap[i] = 0;

   //Realtime tasks
   mlockall (MCL_CURRENT | MCL_FUTURE);
   rt_print_auto_init (1);
   rt_task_shadow (&program, "soem-master", 20, T_JOINABLE);
   rt_task_create (&task, "Send PDO", 8192, 99, 0);	// Create the realtime task, but don't start it yet
   rt_mutex_create (&mutex, "Mutex");
}

EcMaster::~EcMaster()
{
//    reset();
   //must clean memory and delete tasks
   rt_task_delete (&task);
   rt_mutex_delete(&mutex);
   delete[] ecPort;
}

bool EcMaster::preconfigure() throw(EcError)
{
  bool success;
  int size = ethPort.size();
  ecPort = new char[size];
  strcpy (ecPort, ethPort.c_str());

  // initialise SOEM, bind socket to ifname
  if (ec_init(ecPort) > 0)
  {

    std::cout << "ec_init on " << ethPort << " succeeded." << std::endl;

    //Initialise default configuration, using the default config table (see ethercatconfiglist.h)
    if (ec_config_init(FALSE) > 0)
    {
	std::cout << ec_slavecount << " slaves found and configured."<< std::endl;
	std::cout << "Request pre-operational state for all slaves"<< std::endl;

	//
	success = switchState (EC_STATE_PRE_OP);
	if (!success)
	  throw( EcError (EcError::FAIL_SWITCHING_STATE_PRE_OP));

	for (int i = 1; i <= ec_slavecount; i++)
	{
	  EcSlave* driver = EcSlaveFactory::Instance().createDriver(&ec_slave[i]);
	  if (driver)
	  {
	    m_drivers.push_back(driver);
	    std::cout << "Created driver for " << ec_slave[i].name<< ", with address " << ec_slave[i].configadr<< std::endl;
	    //Adding driver's services to master component
	    std::cout << "Put configuration parameters in the slaves."<< std::endl;
	    driver->configure();
	    
	  }else{
	    std::cout << "Could not create driver for "<< ec_slave[i].name << std::endl;
	    throw( EcError (EcError::FAIL_CREATING_DRIVER));
	  }

	}
	//Configure distributed clock
	//ec_configdc();
	//Read the state of all slaves
	//ec_readstate();

    }else{
	std::cout << "Configuration of slaves failed!!!" << std::endl;
	if(EcatError)
    throw(EcError(EcError::ECAT_ERROR));
	return false;

    }
    if(EcatError)
	throw(EcError(EcError::ECAT_ERROR));

  }else{
    std::cout << "Could not initialize master on " << ethPort.c_str() << std::endl;
    return false;

  }
  std::cout<<"Master preconfigured!!!"<<std::endl;
  return true;
}


bool EcMaster::configure() throw(EcError)
{
  bool success;
  int32_t wkc, expectedWKC;
  ec_config_map(&m_IOmap);
  if(EcatError)
    throw(EcError(EcError::ECAT_ERROR));

  std::cout << "Request safe-operational state for all slaves" << std::endl;
  success = switchState (EC_STATE_SAFE_OP);
  if (!success)
    throw(EcError(EcError::FAIL_SWITCHING_STATE_SAFE_OP));

  // send one valid process data to make outputs in slaves happy
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  if(EcatError)
    throw(EcError(EcError::ECAT_ERROR));

  std::cout << "Request operational state for all slaves" << std::endl;
  success = switchState(EC_STATE_OPERATIONAL);
  if (!success)
	throw(EcError(EcError::FAIL_SWITCHING_STATE_OPERATIONAL));

  std::cout<<"Master configured!!!"<<std::endl;

  return true;
}


bool EcMaster::start()
{
   //Starts a preiodic tasck that sends frames to slaves
   rt_task_set_periodic (&task, TM_NOW, m_cycleTime);
   rt_task_start (&task, &ethercatLoop, NULL);


   // Assegurem que els servos estan shutted down
   for (int i = 0 ; i < m_drivers.size() ; i++)
      ((EcSlaveSGDV*) m_drivers[i]) -> writeControlWord(CW_SHUTDOWN);
   usleep (100000);

   // Switch servos ON
   for(int i=0;i<m_drivers.size();i++)
      ((EcSlaveSGDV*) m_drivers[i])->writeControlWord(CW_SWITCH_ON);
   usleep (100000);

   // Enable movement
   for(int i=0;i<m_drivers.size();i++)
      ((EcSlaveSGDV*) m_drivers[i])->writeControlWord(CW_ENABLE_OP);
   usleep (100000);

   std::cout<<"Master started!!!"<<std::endl;

   return true;
}

bool EcMaster::setVelocity (std::vector <int32_t>&vel)
{

if(vel.size()!=3)
   {
   std::cout<<"Vector velocity dimension has to be 3"<<std::endl;
   return false;
   }
   for(int i=0;i<m_drivers.size();i++)
   ((EcSlaveSGDV*) m_drivers[i])->writeVelocity(vel[i]);

   return true;
}
// bool EcMaster::getVelocity (std::vector <int32_t>&vel)
// {
//   vel.resize(3);
//   for(int i=0;i<m_drivers.size();i++)
//     m_drivers[i]->readVelocity(vel[i]);
//
//   return true;
// }

bool EcMaster::stop()
{
   //desactivating motors and ending ethercatLoop
   for(int i=0;i<m_drivers.size();i++)
   ((EcSlaveSGDV*) m_drivers[i])->writeControlWord(CW_SHUTDOWN);
   usleep (100000);

   for(int i=0;i<m_drivers.size();i++)
   ((EcSlaveSGDV*) m_drivers[i])->writeControlWord(CW_SHUTDOWN);
   usleep (100000);

// Aturem la tasca periòdica

rt_task_suspend (&task);
std::cout<<"Master stoped!"<<std::endl;
return true;
}



bool EcMaster::setPosition (std::vector <int32_t>&pos)
{
   return true;//if all is ok
}



bool EcMaster::getPosition (std::vector <int32_t>&pos)
{
   return true;//if all is ok
}


bool EcMaster::reset() throw(EcError)
{
   bool success;
   
   success = switchState (EC_STATE_INIT);
   if (!success)
      throw(EcError(EcError::FAIL_SWITCHING_STATE_INIT));

   for (unsigned int i = 0; i < m_drivers.size(); i++)
         delete m_drivers[i];
   ec_close();
   
   std::cout<<"Master reseted!"<<std::endl;

   return true;
}

//Private functions

//switch the state of state machine
bool EcMaster::switchState (ec_state state)
{
   bool reachState=false;
   /* Request the desired state for all slaves */
   ec_slave[0].state = state;
   ec_writestate (0);
   /* wait for all slaves to reach the desired state */
   ec_statecheck (0, state,  EC_TIMEOUTSTATE * 4);
   //check if all slave reached the desired state
   if (ec_slave[0].state == state)
   {
      reachState = true;
   }
   return reachState;
}

}     ///end of the class soem_master



