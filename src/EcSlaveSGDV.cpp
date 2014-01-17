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
    useDC (true), SYNC0TIME (1000000), SHIFT (125000),SHIFTMASTER (1000000), PDOerrorsTolerance (9),
    recieveEntry(0),transmitEntry(0), 
    controlWordEntry(0), targetPositionEntry(0), targetVelocityEntry(0), targetTorqueEntry(0),
    statusWordEntry(0), actualPositionEntry(0), actualVelocityEntry(0), actualTorqueEntry(0),
    wControlWordCapable(false), wPositionCapable(false), wVelocityCapable(false),wTorqueCapable(false),
    rStatusWordCapable(false), rPositionCapable(false), rVelocityCapable(false), rTorqueCapable(false)
{
 //  parameter temp;
   m_params.resize(0);
   inputObjects.resize(0);
   outputObjects.resize(0);
   m_name = "SGDV_" + to_string(m_datap->configadr & 0x0f,std::dec);  
   

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

const std::string& EcSlaveSGDV::getName() const
{
    return m_name;
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
    int inputSize  = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize;
    int outputSize = outputObjects[outputObjects.size()-1].offset + outputObjects[outputObjects.size()-1].byteSize;
//    std::cout<<"InputSize "<<inputSize<<std::endl;
//    std::cout<<"OutputSize "<<outputSize<<std::endl;
    inputPDO  = new char[inputSize];
    outputPDP = new char[outputSize];
    
    std::cout << getName() << " configured !" <<std::endl;
    
    
    return true;
}

void EcSlaveSGDV::start() throw(EcErrorSGDV)
{
  
  //writePDO(FIRST_ENTRY,CW_SHUTDOWN);
  writeControlWord(CW_SHUTDOWN);
  usleep (100000);
  
 // writePDO(FIRST_ENTRY,CW_SWITCH_ON);
  writeControlWord(CW_SWITCH_ON);
  usleep (100000);
  
  // Enable movement
 // writePDO(FIRST_ENTRY,CW_ENABLE_OP);
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

bool EcSlaveSGDV::writePDO (EcPDOEntry entry, int value)
{
    if(entry < 0 || entry >= outputObjects.size())
    {
        std::cout<<"entry "<<entry<<std::endl;
       	std::cout<<"outputsize "<<outputObjects.size()<<std::endl;
       	throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    }
    //write on the desired position of the PDO
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (m_datap->outputs + outputObjects[entry].offset, &value ,outputObjects[entry].byteSize);
    rt_mutex_release (&mutex);
}

bool EcSlaveSGDV::readPDO (EcPDOEntry entry, int& value)
{
    if(entry<0 || entry>=inputObjects.size())
	throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    //read the desired position of the PDO
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (&value, m_datap->inputs + inputObjects[entry].offset, inputObjects[entry].byteSize);
    rt_mutex_release (&mutex);
}


bool EcSlaveSGDV::writeControlWord (uint16_t controlWord)
{
    if (!wControlWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (m_datap->outputs + outputObjects[controlWordEntry].offset, &controlWord ,outputObjects[controlWordEntry].byteSize);
    rt_mutex_release (&mutex);
}
bool EcSlaveSGDV::readStatusWord (uint16_t &statusWord)
{
    if (!rStatusWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (&statusWord ,m_datap->outputs + inputObjects[statusWordEntry].offset, inputObjects[statusWordEntry].byteSize);
    rt_mutex_release (&mutex);
    
} 
bool EcSlaveSGDV::writePosition (int32_t position)
{
    if (!wPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (m_datap->outputs + outputObjects[targetPositionEntry].offset, &position ,outputObjects[targetPositionEntry].byteSize);
    rt_mutex_release (&mutex);
}
bool EcSlaveSGDV::readPosition (int32_t &position)
{
    if (!rPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (&position ,m_datap->outputs + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    rt_mutex_release (&mutex);
}
bool EcSlaveSGDV::writeVelocity (int32_t velocity)
{
    if (!wVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (m_datap->outputs + outputObjects[targetVelocityEntry].offset, &velocity ,outputObjects[targetVelocityEntry].byteSize);
    rt_mutex_release (&mutex);
}
bool EcSlaveSGDV::readVelocity (int32_t &velocity)
{
    if (!rVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (&velocity ,m_datap->outputs + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    rt_mutex_release (&mutex);
}
bool EcSlaveSGDV::writeTorque (int16_t torque)
{
    if (!wTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (m_datap->outputs + outputObjects[targetTorqueEntry].offset, &torque ,outputObjects[targetTorqueEntry].byteSize);
    rt_mutex_release (&mutex);
}
bool EcSlaveSGDV::readTorque (int16_t &torque)
{
    if (!rTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    rt_mutex_acquire (&mutex, TM_INFINITE);
    memcpy (&torque ,m_datap->outputs + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    rt_mutex_release (&mutex);
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
	std::string PDOentry = std::string(param.attribute("PDOentry").value());
	std::string name = std::string(param.attribute("name").value());
	
      	if (name ==  "description")
	{
	  if (type != "string")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
	  temp.description = param.child_value();
	 
	}
	else if (name ==  "name")
	{
	  if (type != "string")
	    throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
	    temp.name = param.child_value();
	}
	else if (name == "index")
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
	  addPDOobject(PDOentry,temp.param,temp.subindex);
	}
	
	
	else
	 throw(EcErrorSGDV(EcErrorSGDV::XML_STRUCTURE_ERROR,m_slave_nr,getName()));
    }
    m_params.push_back(temp);
  }
  if(outputObjects.size() <= 0)
      throw(EcErrorSGDV(EcErrorSGDV::OUTPUT_OBJECTS_ERROR,m_slave_nr,getName()));
  if(inputObjects.size() <= 0)
      throw(EcErrorSGDV(EcErrorSGDV::INPUT_OBJECTS_ERROR,m_slave_nr,getName()));

  //Calculating offset of outputs and inputs if the entries aren't oredered
//   for (int n = 1; n < inputObjects.size ; n++)
//       inputObjects[n].offset = inputObjects[n-1].offset + inputObjects[n-1].byteSize;
//   for (int n = 1; n < outputObjects.size ; n++)
//       outputObjects[n].offset = outputObjects[n-1].offset + outputObjects[n-1].byteSize;

}

bool EcSlaveSGDV::addPDOobject (std::string PDOentry, int value, int subindex)
{
    if ( ((PDOentry != "transmit") && (PDOentry != "recieve")) || subindex == 0 )
    {
	return false;
    }
    int mask1 = 0xFFFF0000;
    int mask2 = 0x0000FFFF;
    int objectNumber = (value & mask1) >> 16;
    int objectSize = (value & mask2)/8;
    if (PDOentry == "transmit")
    {
	PDOobject temp;
	if(transmitEntry>0)
	{
	  temp.offset = inputObjects[transmitEntry-1].offset + inputObjects[transmitEntry-1].byteSize;
 	}else{
          temp.offset = 0;
	}
	temp.byteSize = objectSize;
	
	switch (objectNumber)
	{
	    case STATUS_WORD:
		statusWordEntry = transmitEntry;
		rStatusWordCapable = true;  
		break;
		
	    case ACTUAL_POSITION:
		actualPositionEntry=transmitEntry;
		rPositionCapable = true; 
		break;
		
	    case ACTUAL_VELOCITY:
		actualVelocityEntry = transmitEntry;
		rVelocityCapable = true;
		break;
		
	    case ACTUAL_TORQUE:
		actualTorqueEntry=transmitEntry;
		rTorqueCapable = true;
		break;
		
	    default:
		break;
		
	}
	inputObjects.push_back(temp);	
	transmitEntry += 1;
    }
    
    if (PDOentry == "recieve")
    {
	PDOobject temp;
	if(recieveEntry>0)
	{
	  temp.offset = outputObjects[recieveEntry-1].offset + outputObjects[recieveEntry-1].byteSize;
        }else{ 	
          temp.offset = 0;
	}
	temp.byteSize = objectSize;
	
	switch (objectNumber)
	{
	    case CONTROL_WORD:
		controlWordEntry = recieveEntry;
		wControlWordCapable = true;  
		break;
		
	    case TARGET_POSITION:
		targetPositionEntry = recieveEntry;
		wPositionCapable = true;
		break;
		
	    case TARGET_VELOCITY:
		targetVelocityEntry = recieveEntry;
		wVelocityCapable = true;
		break;
		
	    case TARGET_TORQUE:
		targetTorqueEntry = recieveEntry;
		wTorqueCapable = true;
		break;
		
	    default:
		break;
		
	}
	outputObjects.push_back(temp);
	recieveEntry += 1;
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

