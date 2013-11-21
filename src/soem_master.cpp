#include "soem_master.hpp"

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
#include "soem_SGDV.hpp"



using namespace std;

namespace servos
{

SoemMaster::SoemMaster() : ethPort ("rteth0")
{    
    cycletime = 1000000;//The cycletime of the updateHook. The time beween each PDO reading and writing.
    opMode = 3;		//Initialising necessary parameters of OpMode 3
    maxPvel = 20000;	// Max. rotational speed is 200 degrees/second
    Pacc = 150;  	// acceleration of 1 degree/sec²
    Pdec = 150; 	// deacceleration: the same
    Pqdec = 50000;	// quick deacceleration: the same
    
    
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

SoemMaster::~SoemMaster()
{	
    //must clean memory and delete tasks
    rt_task_delete (&task);
    rt_mutex_delete(&mutex);
    delete[] ecPort;
}

bool SoemMaster::preconfigure()
{
    bool success;
    int32_t wkc, expectedWKC;
    int size = ethPort.size();
    ecPort = new char[size];
    strcpy (ecPort, ethPort.c_str());
    
    // initialise SOEM, bind socket to ifname
    if (ec_init(ecPort) > 0)
    {
      cout << "ec_init on " << ethPort << " succeeded." << endl;
      
      //Initialise default configuration, using the default config table (see ethercatconfiglist.h)
      if (ec_config_init(FALSE) > 0)
      {
	//ec_config_map(&m_IOmap);
	
	cout << ec_slavecount << " slaves found and configured."
                    << endl;
            cout << "Request pre-operational state for all slaves"
                    << endl;
	
	success = switchState (EC_STATE_PRE_OP);
	if (!success) 
	{
	  return false;
	} else {
	  std::cout << "Reached pre-operational" << std::endl;
	}
	for (int i = 1; i <= ec_slavecount; i++)
	{
	  SoemDriver* driver = SoemDriverFactory::Instance().createDriver(&ec_slave[i]);
	  if (driver)
          {
	    m_drivers.push_back(driver);
	    cout << "Created driver for " << ec_slave[i].name
                      << ", with address " << ec_slave[i].configadr<< endl;
	    //Adding driver's services to master component
	    cout << "Put configuration parameters in the slaves."<< endl;
	    if (!driver->configure())
	      return false;
	    
	  }else{
	    cout << "Could not create driver for "<< ec_slave[i].name << endl;
	    
	  }
	  
	}
	//Configure distributed clock
        //ec_configdc();
        //Read the state of all slaves

            //ec_readstate();

      }else{
	
            cout << "Configuration of slaves failed!!!" << endl;
            return false;
        }
        while (EcatError)
            {
                cout << ec_elist2string() << endl;
            }
        
        return true;
    }
    else
    {
        cout << "Could not initialize master on " << ethPort.c_str() << endl;
        return false;
    }
    cout<<"Master preconfigured!!!"<<endl;
    return true;
}


bool SoemMaster::configure()
{
  bool success;
  ec_config_map(&m_IOmap);
  while (EcatError)
  {
    cout << ec_elist2string() << endl;
    
  }
  cout << "Request safe-operational state for all slaves" << endl;
  success = switchState (EC_STATE_SAFE_OP);
  if (!success) 
  {
    return false;
    
  } else {
    std::cout << "Reached safe-operational" << std::endl;
    
  }
  // send one valid process data to make outputs in slaves happy
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
        
  cout << "Request operational state for all slaves" << endl;
  success = switchState(EC_STATE_OPERATIONAL);
  
  
  if (!success) 
  {
    return false;
    
  } else {
    std::cout << "Reached operational" << std::endl;
    
  }
  cout<<"Master configured!!!"<<endl;

  return true;    
}


bool SoemMaster::start()
{
    //Starts a preiodic tasck that sends frames to slaves
    rt_task_set_periodic (&task, TM_NOW, cycletime);
    rt_task_start (&task, &ethercatLoop, NULL);
    while (EcatError)
    {
      cout << ec_elist2string() << endl;        
    }
    cout<<"Switching on motors..."<<endl;
    
     // Assegurem que els servos estan shutted down
     for (int i = 0 ; i < m_drivers.size() ; i++)
      ((SoemSGDV*) m_drivers[i]) -> writeControlWord(CW_SHUTDOWN);
     usleep (100000);
     
     // Switch servos ON
     for(int i=0;i<m_drivers.size();i++)
      ((SoemSGDV*) m_drivers[i])->writeControlWord(CW_SWITCH_ON);
     usleep (100000);
     
     // Enable movement
     for(int i=0;i<m_drivers.size();i++)
       ((SoemSGDV*) m_drivers[i])->writeControlWord(CW_ENABLE_OP);
     usleep (100000);
     
//     cout << "Motors should be ON" << endl;

    cout<<"Master started!!!"<<endl;
    
    return true;//if all is ok
}

bool SoemMaster::setVelocity (std::vector <int32_t>&vel)
{
  
  if(vel.size()!=3)
   {
     cout<<"Vector velocity dimension has to be 3"<<endl;
     return false;
   }
   for(int i=0;i<m_drivers.size();i++)
     ((SoemSGDV*) m_drivers[i])->writeVelocity(vel[i]);
   
   return true;
}
// bool SoemMaster::getVelocity (std::vector <int32_t>&vel)
// {
//   vel.resize(3);
//   for(int i=0;i<m_drivers.size();i++)
//     m_drivers[i]->readVelocity(vel[i]);
//   
//   return true;
// }

bool SoemMaster::stop()
{
   //desactivating motors and ending ethercatLoop
   for(int i=0;i<m_drivers.size();i++)
     ((SoemSGDV*) m_drivers[i])->writeControlWord(CW_SHUTDOWN);
   usleep (100000);
   
   for(int i=0;i<m_drivers.size();i++)
     ((SoemSGDV*) m_drivers[i])->writeControlWord(CW_SHUTDOWN);
   usleep (100000);

  // Aturem la tasca periòdica

  rt_task_suspend (&task);
  cout<<"Master stoped!"<<endl;
  return true;
}



bool SoemMaster::setPosition (std::vector <int32_t>&pos)
{
    return true;//if all is ok
}



bool SoemMaster::getPosition (std::vector <int32_t>&pos)
{
    return true;//if all is ok
}


bool SoemMaster::reset()
{
    bool success;
    cout<<"Reseting...."<<endl;
    success = switchState (EC_STATE_SAFE_OP);
    success = switchState (EC_STATE_PRE_OP);
    success = switchState (EC_STATE_INIT);
    
    for (unsigned int i = 0; i < m_drivers.size(); i++)
          delete m_drivers[i];
//    ec_close();      
    cout<<"Should be reseted!"<<endl;
    if (!success) {
         cout<<"Not achieved Init state reseting"<<endl;
        return false;
    }     
    return true;
}

//--internal functions(private)--

// This will be the main communication loop to be executed in a realtime separate thread

bool SoemMaster::switchState (int state)
{
    bool reachState;
    cout<<"Desired state: "<<state<<endl;
    //switch the state of state machine
    /* Request the desired state for all slaves */
    ec_slave[0].state = state;
    ec_writestate (0);
    /* wait for all slaves to reach the desired state */
    ec_statecheck (0, state,  EC_TIMEOUTSTATE * 4);
    //check if all slave reached the desired state
    if (ec_slave[0].state == state)
      {
	cout << "Desided state reached for all slaves."<< endl;
	reachState = true;
      }else{
	cout << "Not all slaves reached desired state."<< endl;
	 //If not all slaves operational find out which one
	for (int i = 1; i <= ec_slavecount; i++)
	{
	  if (ec_slave[i].state != state)
	  {
	    cout << "Slave " << i 
		       << " State= " << to_string(ec_slave[i].state, std::hex) 
		       << " StatusCode=" << ec_slave[i].ALstatuscode << " : "
		       << ec_ALstatuscode2string(ec_slave[i].ALstatuscode) 
		       << endl;
	    
	  }
	  
	}
	reachState = false;
      }
      
    return reachState;
}

}     ///end of the class soem_master



