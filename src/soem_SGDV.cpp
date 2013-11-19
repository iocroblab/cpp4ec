#include "soem_SGDV.hpp"

#include <sys/mman.h>
#include "soem_driver_factory.h"



using namespace std;


namespace servos
{
SoemSGDV::SoemSGDV (ec_slavet* mem_loc) : SoemDriver (mem_loc),
    useDC (true), SYNC0TIME (1000000), SHIFT (125000), 
    SHIFTMASTER (1000000), PDOerrorsTolerance (9)
{
    rt_mutex_create (&mutex, "Mutex");
  
   parameter temp;
   //setting parameters 
   temp.description = "Modes of Operation";
   temp.index = 0x6060;
   temp.subindex = 0x00;
   temp.name = "OpMode";
   temp.size = 1;
   temp.param = 3;
   m_params.push_back(temp);
   
   temp.description = "Max. Profile velocity";
   temp.index = 0x607F;
   temp.subindex = 0x00;
   temp.name = "maxPvel";
   temp.size = 4;
   temp.param = 2000;
   m_params.push_back(temp);
   
   temp.description = "Profile acceleration";
   temp.index = 0x6083;
   temp.subindex = 0x00;
   temp.name = "Pacc";
   temp.size = 4;
   temp.param = 150;
   m_params.push_back(temp);
   
   //PDO mapping
   //RxPDO
   //uint32_t ControlWord = 0x60400010;
   //uint32_t TargetPosition=0x607A0020;
   //uint32_t TargetVelovity = 0x60FF0020;
   //uint32_t TargetToque=0x60710010;
   //uint32_t MaxTorque=0x60720010;
   //uint32_t ModeOperation=0x60600008;
   //TxPDO
   //uint32_t StatusWord = 0x60410010;
   //uint32_t PositionActual = 0x60640020;
   //uint32_t VelocityActual = 0x606C0020;
   //uint32_t TorqueActual=0x60770010;
   //uint32_t ErrorActual=0x60F40020;
   //uint32_t ModeActual = 0x60610008
   
   //1.Disable the assignment of the Sync manager and PDO   
   temp.description = "Disable";
   temp.index = 0x1C12;
   temp.subindex = 0x00;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 1;
   temp.param = 0;
   m_params.push_back(temp);
   
   temp.description = "Disable";
   temp.index = 0x1C13;
   temp.subindex = 0x00;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 1;
   temp.param = 0;
   m_params.push_back(temp);
   
   //2.Set all the mapping entry in PDO mapping objects   
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1602;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 0;
   m_params.push_back(temp);
   
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1602;
   temp.subindex = 0x01;
   temp.name = "Control Word ";
   temp.size = 4;
   temp.param = 0x60400010;
   m_params.push_back(temp);
   
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1602;
   temp.subindex = 0x02;
   temp.name = "Target Velovity ";
   temp.size = 4;
   temp.param = 0x60FF0020;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A02;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 0;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A02;
   temp.subindex = 0x01;
   temp.name = "Status Word";
   temp.size = 4;
   temp.param = 0x60410010;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A02;
   temp.subindex = 0x01;
   temp.name = "Position Actual Value";
   temp.size = 4;
   temp.param = 0x60640020;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A02;
   temp.subindex = 0x02;
   temp.name = "Velocity Actual Value";
   temp.size = 4;
   temp.param = 0x606C0020;
   m_params.push_back(temp);
   
   //3.Set the number of mapping entries in PDO mapping objects 
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1602;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 2;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A02;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 3;
   m_params.push_back(temp);
   
   //4.Set the assignment of the Sync manager and PDO    
   temp.description = "Assing";
   temp.index = 0x1C12;
   temp.subindex = 0x01;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 2;
   temp.param = 0x1602;
   m_params.push_back(temp);
   
   temp.description = "Assing";
   temp.index = 0x1C13;
   temp.subindex = 0x01;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 2;
   temp.param = 0x1A02;
   m_params.push_back(temp);
   
   //5.Enable the assignment of the Sync manager and PDO.
   temp.description = "Enable";
   temp.index = 0x1C12;
   temp.subindex = 0x00;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 1;
   temp.param = 1;
   m_params.push_back(temp);
   
   temp.description = "Enable";
   temp.index = 0x1C13;
   temp.subindex = 0x00;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 1;
   temp.param = 1;
   m_params.push_back(temp);

}

SoemSGDV::~SoemSGDV()
{
  rt_mutex_delete(&mutex);
}



bool SoemSGDV::configure()
{
    for (unsigned int i = 0; i < m_params.size(); i++) 
    {
      while (EcatError)
	cout << ec_elist2string() << endl;
      
      ec_SDOwrite(m_slave_nr, m_params[i].index, m_params[i].subindex, FALSE, 
		  m_params[i].size,&(m_params[i].param),EC_TIMEOUTRXM);      
      
    }

//     if (useDC) {
//         //To change the error tolerance before starting
//         SDOwrite (0x1F01, 2, false, 4, PDOerrorsTolerance);
//         //To set the desired shift of the outputs in the slave
//         SDOwrite (0x1C33, 0x03, false, 4, SHIFT);
//         //To let know the master that this slave will use DC
//         configmyDC = this->getPeer ("Master")->getOperation ("configDC");
//         configmyDC (internalNumber, true, SYNC0TIME, SHIFTMASTER);
//     }
    std::cout << getName() << " configured !" <<std::endl;
    return true;
}

bool SoemSGDV::writeControlWord (uint16_t controlWord)
{
    //switch the motor state
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (m_datap->outputs, &controlWord , 2);
    rt_mutex_release (&mutex);
}

bool SoemSGDV::readStatusWord (uint16_t statusWord)
{
    //switch the motor state
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (&statusWord, m_datap->inputs, 2);
    rt_mutex_release (&mutex);
}

bool SoemSGDV::writeVelocity (int32_t velocity)
{

    rt_mutex_acquire (&mutex, TM_INFINITE);
    //velocity starts in byte 6 (2bytes Controlword + 4bytes TargetPosition)
    memcpy (m_datap->outputs + 2, &velocity, 4);
    rt_mutex_release (&mutex);
    return true;//if all is ok
}

bool SoemSGDV::readVelocity (int32_t velocity)
{

    rt_mutex_acquire (&mutex, TM_INFINITE);
    //velocity starts in byte 6 (2bytes Controlword + 4bytes TargetPosition)
    memcpy (&velocity, m_datap->inputs + 6, 4);
    rt_mutex_release (&mutex);
    return true;//if all is ok
}


namespace {
servos::SoemDriver* createSoemSGDV(ec_slavet* mem_loc) {
	return new SoemSGDV(mem_loc);
}

const bool registered0 = servos::SoemDriverFactory::Instance().registerDriver("? M:00000539 I:02200001", createSoemSGDV);

}
}

