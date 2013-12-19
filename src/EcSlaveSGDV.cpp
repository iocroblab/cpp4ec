#include "EcSlaveSGDV.h"

#include <sys/mman.h>
#include "EcSlaveSGDV.h"
#include "EcSlaveFactory.h"

//Xenomai
#include <native/task.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>

#include <pugixml.hpp>
#include <stdint.h>
#include <stdlib.h> 

/*
#include <sys/time.h>
#include <time.h>*/

#include <unistd.h>




namespace cpp4ec
{

extern RT_MUTEX mutex;

EcSlaveSGDV::EcSlaveSGDV (ec_slavet* mem_loc) : EcSlave (mem_loc),
    useDC (true), SYNC0TIME (1000000), SHIFT (125000),
    SHIFTMASTER (1000000), PDOerrorsTolerance (9)
{
   parameter temp;
   
   readXML();
   
//   setting parameters
//    temp.description = "Modes of Operation";
//    temp.index = 0x6060;
//    temp.subindex = 0x00;
//    temp.name = "OpMode";
//    temp.size = 1;
//    temp.param = 3;
//    m_params.push_back(temp);
//    
//    //Position parameter (mode = 1)
//    temp.description = "Min Position Limit";
//    temp.index = 0x607D;
//    temp.subindex = 0x01;
//    temp.name = "MinPos";
//    temp.size = 4;
//    temp.param = -80000; 
//    m_params.push_back(temp);
// 
//    temp.description = "Max Position Limit";
//    temp.index = 0x607D;
//    temp.subindex = 0x02;
//    temp.name = "MaxPos";
//    temp.size = 4;
//    temp.param = 80000;  
//    m_params.push_back(temp);
// 
//    temp.description = "Profile velocity";
//    temp.index = 0x6081;
//    temp.subindex = 0x00;
//    temp.name = "Pvel"; 
//    temp.size = 4;
//    temp.param = 150000;
//    m_params.push_back(temp);
// 
//    //Velocity Parameters (mode = 3,9)
//    
//    temp.description = "Max. Profile velocity";
//    temp.index = 0x607F;
//    temp.subindex = 0x00;
//    temp.name = "maxPvel";
//    temp.size = 4;
//    temp.param = 150000;
//    m_params.push_back(temp);
// 
//    temp.description = "Profile acceleration";
//    temp.index = 0x6083;
//    temp.subindex = 0x00;
//    temp.name = "Pacc";  
//    temp.size = 4;
//    temp.param = 150;
//    m_params.push_back(temp);
// 
//    //Torque parameters (mode = 4)
//    temp.description = "Max Torque";
//    temp.index = 0x6072;
//    temp.subindex = 0x00;
//    temp.name = "MaxTorq";
//    temp.size = 2;
//    temp.param = 1000;
//    m_params.push_back(temp);
// 
//    temp.description = "Slope Torque";
//    temp.index = 0x6087;
//    temp.subindex = 0x00;
//    temp.name = "STorq"; 
//    temp.size = 4;
//    temp.param = 100;
//    m_params.push_back(temp);
// 
//    /*-----PDO Mapping-----*/
//    //1.Disable the assignment of the Sync manager and PDO
//    temp.description = "Disable";
//    temp.index = 0x1C12;
//    temp.subindex = 0x00;
//    temp.name = "Sync Manager PDO Assignment";
//    temp.size = 1;
//    temp.param = 0;
//    m_params.push_back(temp);
// 
//    temp.description = "Disable";
//    temp.index = 0x1C13;
//    temp.subindex = 0x00;
//    temp.name = "Sync Manager PDO Assignment";
//    temp.size = 1;
//    temp.param = 0;
//    m_params.push_back(temp);
// 
//    //2.Set all the mapping entry in PDO mapping objects
//      temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600;
//    temp.subindex = 0x00;
//    temp.name = "Number of objects";
//    temp.size = 1;
//    temp.param = 0;
//    m_params.push_back(temp);
// 
//    temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600;
//    temp.subindex = 0x01;
//    temp.name = "Control Word ";
//    temp.size = 4;
//    temp.param = 0x60400010;
//    m_params.push_back(temp);
// 
//    temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600;
//    temp.subindex = 0x02;
//    temp.name = "Target Position ";
//    temp.size = 4;
//    temp.param = 0x607A0020;
//    m_params.push_back(temp);
// 
//    temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600;
//    temp.subindex = 0x03;
//    temp.name = "Target Velovity ";
//    temp.size = 4;
//    temp.param = 0x60FF0020;
//    m_params.push_back(temp);
// 
//    temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600; 
//    temp.subindex = 0x04;
//    temp.name = "Target Torque ";
//    temp.size = 4; 
//    temp.param = 0x60710010;
//    m_params.push_back(temp);
// 
// /*   temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600; 
//    temp.subindex = 0x05;
//    temp.name = "Max Torque ";
//    temp.size = 4; 
//    temp.param = 0x60720010;
//    m_params.push_back(temp);
// */                     
//    temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600; 
//    temp.subindex = 0x05;
//    temp.name = "Mode of Operation ";
//    temp.size = 4; 
//    temp.param = 0x60600008;
//    m_params.push_back(temp);
// 
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00;
//    temp.subindex = 0x00;
//    temp.name = "Number of objects";
//    temp.size = 1;
//    temp.param = 0;
//    m_params.push_back(temp);
// 
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00;
//    temp.subindex = 0x01;
//    temp.name = "Status Word";
//    temp.size = 4;
//    temp.param = 0x60410010;
//    m_params.push_back(temp);
// 
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00;
//    temp.subindex = 0x01;
//    temp.name = "Position Actual Value";
//    temp.size = 4;
//    temp.param = 0x60640020;
//    m_params.push_back(temp);
// 
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00;
//    temp.subindex = 0x02;
//    temp.name = "Velocity Actual Value";
//    temp.size = 4;
//    temp.param = 0x606C0020;
//    m_params.push_back(temp);
// 
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00; 
//    temp.subindex = 0x04;
//    temp.name = "Torque Actual Value";
//    temp.size = 4; 
//    temp.param = 0x60770010;
//    m_params.push_back(temp);
// 
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00; 
//    temp.subindex = 0x05;
//    temp.name = "Error Code";
//    temp.size = 4; 
//    temp.param = 0x603F0010;
//    m_params.push_back(temp);
//                             
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00; 
//    temp.subindex = 0x06;
//    temp.name = "Mode of Operation Display";
//    temp.size = 4; 
//    temp.param = 0x60610008;
//    m_params.push_back(temp);
// 
// 
//    //3.Set the number of mapping entries in PDO mapping objects
//    temp.description = "Receive PDO Mapping";
//    temp.index = 0x1600;
//    temp.subindex = 0x00;
//    temp.name = "Number of objects";
//    temp.size = 1;
//    temp.param = 5;
//    m_params.push_back(temp);
// 
//    temp.description = "Transmit PDO Mapping";
//    temp.index = 0x1A00;
//    temp.subindex = 0x00;
//    temp.name = "Number of objects";
//    temp.size = 1;
//    temp.param = 6;
//    m_params.push_back(temp);
// 
//    //4.Set the assignment of the Sync manager and PDO
//    temp.description = "Assing";
//    temp.index = 0x1C12;
//    temp.subindex = 0x01;
//    temp.name = "Sync Manager PDO Assignment";
//    temp.size = 2;
//    temp.param = 0x1600;
//    m_params.push_back(temp);
// 
//    temp.description = "Assing";
//    temp.index = 0x1C13;
//    temp.subindex = 0x01;
//    temp.name = "Sync Manager PDO Assignment";
//    temp.size = 2;
//    temp.param = 0x1A00;
//    m_params.push_back(temp);
// 
//    //5.Enable the assignment of the Sync manager and PDO.
//    temp.description = "Enable";
//    temp.index = 0x1C12;
//    temp.subindex = 0x00;
//    temp.name = "Sync Manager PDO Assignment";
//    temp.size = 1;
//    temp.param = 1;
//    m_params.push_back(temp);
// 
//    temp.description = "Enable";
//    temp.index = 0x1C13;
//    temp.subindex = 0x00;
//    temp.name = "Sync Manager PDO Assignment";
//    temp.size = 1;
//    temp.param = 1;
//    m_params.push_back(temp);
   
}

EcSlaveSGDV::~EcSlaveSGDV()
{
}

bool EcSlaveSGDV::configure() throw(EcErrorSGDV)
{
    for (unsigned int i = 0; i < m_params.size(); i++)
    {
      ec_SDOwrite(m_slave_nr, m_params[i].index, m_params[i].subindex, FALSE,
		  m_params[i].size,&(m_params[i].param),EC_TIMEOUTRXM);
      if(EcatError)
	throw(EcErrorSGDV(EcErrorSGDV::ECAT_ERROR,m_slave_nr,getName()));

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

void EcSlaveSGDV::start() throw(EcErrorSGDV)
{
  
  writeControlWord(CW_SHUTDOWN);
  usleep (100000);
  
  writeControlWord(CW_SWITCH_ON);
  usleep (100000);
  
  // Enable movement
  writeControlWord(CW_ENABLE_OP);
  usleep (100000);
}


void EcSlaveSGDV::stop() throw(EcErrorSGDV)
{
  writeControlWord(CW_SHUTDOWN);
  usleep (100000);
  
  writeControlWord(CW_QUICK_STOP);
  usleep (100000);
}

bool EcSlaveSGDV::writeControlWord (EcControlWord controlWord)
{
    //switch the motor state
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (m_datap->outputs, &controlWord , 2);
    rt_mutex_release (&mutex);
}

bool EcSlaveSGDV::readStatusWord (EcStatusWord statusWord)
{
    //switch the motor state
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (&statusWord, m_datap->inputs, 2);
    rt_mutex_release (&mutex);
}

bool EcSlaveSGDV::writeVelocity (int32_t velocity)
{

    rt_mutex_acquire (&mutex, TM_INFINITE);
    //velocity starts in byte 6 (2bytes Controlword + 4bytes TargetPosition)
    memcpy (m_datap->outputs + 6, &velocity, 4);
    rt_mutex_release (&mutex);
    return true;//if all is ok
}

bool EcSlaveSGDV::readVelocity (int32_t velocity)
{

    rt_mutex_acquire (&mutex, TM_INFINITE);
    //velocity starts in byte 6 (2bytes Controlword + 4bytes TargetPosition)
    memcpy (&velocity, m_datap->inputs + 6, 4);
    rt_mutex_release (&mutex);
    return true;//if all is ok
}



void EcSlaveSGDV::readXML() throw(EcErrorSGDV)
{
  parameter temp;  
  std::string xml_name = "configure_SGDV_"+to_string(m_slave_nr,std::dec)+".xml";
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(xml_name.c_str());
  if (!result)
    throw(EcErrorSGDV(EcErrorSGDV::XML_NOT_FOUND_ERROR,m_slave_nr,getName()));  

  pugi::xml_node parameters = doc.first_child();
  for (pugi::xml_node structure = parameters.first_child(); structure; structure = structure.next_sibling())
  {
    for (pugi::xml_node param = structure.first_child(); param; param = param.next_sibling())
    {
        std::string type = std::string(param.attribute("type").value());
	std::string name = std::string(param.attribute("name").value());
	
      	if (name == "index")
	{
	  if (type != "integer")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));	  
	  temp.index = (int16_t) strtol (param.child_value(),NULL,0);          
	}
	else if (name ==  "subindex")
	{
	  if (type != "integer")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
	  temp.subindex = (int8_t) strtol (param.child_value(),NULL,0);
	}
	else if (name ==  "size")
	{
	  if (type != "integer")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));	  
	  temp.size = (int8_t) strtol (param.child_value(),NULL,0);
	}
	else if (name ==  "value")
	{
	  if (type != "integer")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
	  temp.param = strtol (param.child_value(),NULL,0);
	}
	else if (name ==  "name")
	{
	  if (type != "string")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
	    temp.name = param.child_value();
	}
	else if (name ==  "description")
	{
	  if (type != "string")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
	  temp.description = param.child_value();
	}
	else
	 throw(EcErrorSGDV(EcErrorSGDV::XML_STRUCTURE_ERROR,m_slave_nr,getName()));
    }
    m_params.push_back(temp);
  }
}

void EcSlaveSGDV::setSGDVOject(uint16_t index, uint8_t subindex, int psize, void * param)
{
  ec_SDOwrite(m_slave_nr, index, subindex, FALSE,psize,param,EC_TIMEOUTRXM);
}

void EcSlaveSGDV::getSGDVObject(uint16_t index, uint8_t subindex, int *psize, void *param)
{
  ec_SDOread(m_slave_nr, index, subindex, FALSE,psize,param,EC_TIMEOUTRXM);
}

namespace {
cpp4ec::EcSlave* createEcSlaveSGDV(ec_slavet* mem_loc)
{
	return new EcSlaveSGDV(mem_loc);
}

const bool registered0 = cpp4ec::EcSlaveFactory::Instance().registerDriver("? M:00000539 I:02200001", createEcSlaveSGDV);

}
}

