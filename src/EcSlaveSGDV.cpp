#include "EcSlaveSGDV.h"
#include "EcUtil.h"

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
extern std::mutex masterMutex;


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

   pBufferOut = NULL;
   pBufferIn = NULL;
   

   readXML();
   

}

EcSlaveSGDV::~EcSlaveSGDV()
{
    delete[] inputPDO;
    delete[] outputPDO;
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
    inputSize  = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize;
    outputSize = outputObjects[outputObjects.size()-1].offset + outputObjects[outputObjects.size()-1].byteSize;


    
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

void EcSlaveSGDV::setPDOBuffer(char * input, char * output)
{
    pBufferIn=input;
    pBufferOut=output;
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
    slaveMutex.lock();
    memcpy (pBufferOut + outputObjects[entry].offset, &value ,outputObjects[entry].byteSize);
    slaveMutex.unlock();
}

bool EcSlaveSGDV::readPDO (EcPDOEntry entry, int& value)
{
    if(entry<0 || entry>=inputObjects.size())
	throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    //read the desired position of the PDO
    slaveMutex.lock();
    memcpy (&value, pBufferIn + inputObjects[entry].offset, inputObjects[entry].byteSize);
    slaveMutex.unlock();
}


bool EcSlaveSGDV::writeControlWord (uint16_t controlWord)
{
    if (!wControlWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (pBufferOut + outputObjects[controlWordEntry].offset, &controlWord ,outputObjects[controlWordEntry].byteSize);
    slaveMutex.unlock();
}
bool EcSlaveSGDV::readStatusWord (uint16_t &statusWord)
{
    if (!rStatusWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (&statusWord ,pBufferIn + inputObjects[statusWordEntry].offset, inputObjects[statusWordEntry].byteSize);
    slaveMutex.unlock();
    
} 
bool EcSlaveSGDV::writePosition (int32_t position)
{
    if (!wPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (pBufferOut + outputObjects[targetPositionEntry].offset, &position ,outputObjects[targetPositionEntry].byteSize);
    slaveMutex.unlock();
}
bool EcSlaveSGDV::readPosition (int32_t &position)
{
    if (!rPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (&position ,pBufferIn + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    slaveMutex.unlock();
}
bool EcSlaveSGDV::writeVelocity (int32_t velocity)
{
    if (!wVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (pBufferOut + outputObjects[targetVelocityEntry].offset, &velocity ,outputObjects[targetVelocityEntry].byteSize);
    slaveMutex.unlock();
}
bool EcSlaveSGDV::readVelocity (int32_t &velocity)
{
    if (!rVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (&velocity ,pBufferIn + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    slaveMutex.unlock();
}
bool EcSlaveSGDV::writeTorque (int16_t torque)
{
    if (!wTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (pBufferOut + outputObjects[targetTorqueEntry].offset, &torque ,outputObjects[targetTorqueEntry].byteSize);
    slaveMutex.unlock();
}
bool EcSlaveSGDV::readTorque (int16_t &torque)
{
    if (!rTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveMutex.lock();
    memcpy (&torque ,pBufferIn + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    slaveMutex.unlock();
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

