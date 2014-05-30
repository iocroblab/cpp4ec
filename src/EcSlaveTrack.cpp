#include "EcSlaveTrack.h"
#include "EcUtil.h"

#include <sys/mman.h>
#include "EcSlaveTrack.h"
#include "EcSlaveFactory.h"

#include <pugixml.hpp>
#include <stdint.h>
#include <stdlib.h> 
#include <unistd.h>

#include <bitset>
#define timestampSize 8

namespace cpp4ec
{

EcSlaveTrack::EcSlaveTrack (ec_slavet* mem_loc) : EcSlave (mem_loc),
    useDC (true), SYNC0TIME (1000000), SHIFT (125000),SHIFTMASTER (1000000), PDOerrorsTolerance (9),
    recieveEntry(0),transmitEntry(0), 
    controlWordEntry(0), targetPositionEntry(0), targetVelocityEntry(0), targetTorqueEntry(0),
    statusWordEntry(0), actualPositionEntry(0), actualVelocityEntry(0), actualTorqueEntry(0),
    wControlWordCapable(false), wPositionCapable(false), wVelocityCapable(false),wTorqueCapable(false),
    rStatusWordCapable(false), rPositionCapable(false), rVelocityCapable(false), rTorqueCapable(false),
    pBufferOut(NULL),pBufferIn(NULL),m_mutex(m_slave_nr-1),inputBuf(NULL),outputBuf(NULL),outputSize(0),
    inputSize(0),minPosition(0),maxPosition(0)
{
   m_params.resize(0);
   inputObjects.resize(0);
   outputObjects.resize(0);
   bufferList.resize(0);
   m_name = "Carril_" + to_string(m_datap->configadr & 0x0f,std::dec);  

   readXML();
}

EcSlaveTrack::~EcSlaveTrack()
{
    for( int i = 0; i < bufferList.size(); i++)
           delete[] bufferList[i];
    delete[] inputBuf;
    delete[] inputNull;

    
}

void EcSlaveTrack::update()
{

    int position, controlword;
    int num = memcmp(pBufferIn,inputNull,inputSize-timestampSize);
    if(num != 0)
    {
        //std::cout<<pBufferIn<<std::endl;

        slaveInMutex.lock();
        memcpy(inputBuf,pBufferIn, inputSize);
        slaveInMutex.unlock();
        readAT(SECOND_ENTRY,position);
        if (position>maxPosition || position<minPosition)
        {
            controlword=49152;
            //controlword=24576;
            writeMDT(FIRST_ENTRY,controlword);
            std::cout<<"halt position "<< position<<  " Max position "<< maxPosition << " Min poisition "<< minPosition << std::endl;

        }

    }else{
        bool print=false;
        if (print)
        {
            for(int i = 0; i < inputSize-timestampSize; i++)
                std::cout<<(int)*(pBufferIn+i);
            std::cout<<std::endl;
        }
        //int acce=286331153;
        //writeMDT(FOURTH_ENTRY,acce);
    }

}

const std::string& EcSlaveTrack::getName() const
{
    return m_name;
}


void EcSlaveTrack::DefaultParameters()
{
    for (unsigned int i = 0; i < m_params.size(); i++)
       {
           ec_SoEwrite(m_slave_nr, 0, m_params[i].elementflags, m_params[i].idn, m_params[i].size, &(m_params[i].param),EC_TIMEOUTRXM);
           if(EcatError)
               throw(EcErrorSGDV(EcErrorSGDV::ECAT_ERROR,m_slave_nr,getName()));

       }
}

bool EcSlaveTrack::configure() throw(EcErrorSGDV)
{

    //Creating the AT telegram
    std::vector<int> idn16;
    idn16 = readIDNofIDNs(16,false);
    int i;
    int ATsize=1+idn16[0]/2; //number of IDN in the AT telegram
    inputObjects.resize(ATsize);
    int sizeIDN=2; //Size of all the controlwords 2Bytes
    int totalsize=sizeIDN;
    inputObjects[0].offset = 0;
    inputObjects[0].byteSize = 2;
    int value;
    for (i=1; i<ATsize;i++)
    {
        sizeIDN=8; //valor of this size is not important but it has to be bigger than the max size of an IDN
        inputObjects[i].offset = totalsize;
        ec_SoEread(1,0, 0x40, idn16[i+1], &sizeIDN, &value, EC_TIMEOUTRXM);
        inputObjects[i].byteSize = sizeIDN;
        totalsize=totalsize+sizeIDN;
    }

    //Creating the MDT telegram
    std::vector<int> idn24;
    idn24 = readIDNofIDNs(24,false);
    //Creating the MDT telegram
    int MDTsize=1+idn24[0]/2; //number of IDN in the MDT telegram
    outputObjects.resize(MDTsize);
    sizeIDN=2; //Size of all the controlwords 2Bytes
    totalsize=sizeIDN;
    outputObjects[0].offset = 0;
    outputObjects[0].byteSize = 2;
    for (i=1; i<MDTsize;i++)
    {
        sizeIDN=8; //valor of this size is not important but it has to be bigger than the max size of a normal IDN
        outputObjects[i].offset = totalsize;
        ec_SoEread(1,0, 0x40, idn24[i+1], &sizeIDN, &value, EC_TIMEOUTRXM);
        outputObjects[i].byteSize = sizeIDN;
        totalsize=totalsize+sizeIDN;
    }

    int size=4;
    ec_SoEread(m_slave_nr, 0, 0x40, 49, &size, &maxPosition,EC_TIMEOUTRXM);
    ec_SoEread(m_slave_nr, 0, 0x40, 50, &size, &minPosition,EC_TIMEOUTRXM);


    inputSize  = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize +timestampSize;
    outputSize = outputObjects[outputObjects.size()-1].offset + outputObjects[outputObjects.size()-1].byteSize;
    inputBuf = new char[inputSize];
    inputNull = new char[inputSize];
    //outputBuf = new char[outputSize];
    //memset(outputBuf,0, outputSize);
    memset(inputBuf,0, inputSize);
    memset(inputNull,0, inputSize);

    std::cout << getName() << " configured !" <<std::endl;
    
    return true;
}
	
void EcSlaveTrack::start() throw(EcErrorSGDV)
{
    int position;
    readAT(SECOND_ENTRY,position);

    int controlWord = 57344;
    writeMDT(SECOND_ENTRY,position);
    writeMDT(FIRST_ENTRY,controlWord);
    updateMaster();
}

void EcSlaveTrack::setPDOBuffer(char * input, char * output)
{
    pBufferIn=input;
    pBufferOut=output;
}

void EcSlaveTrack::stop() throw(EcErrorSGDV)
{
    int controlWord = 24576;
    writeMDT(FIRST_ENTRY,controlWord);
    updateMaster();

}

void EcSlaveTrack::setDC(bool active, unsigned int sync0Time, unsigned int sync0Shift) throw(EcErrorSGDV)
{
    ec_dcsync0(m_slave_nr, active, sync0Time, sync0Shift);// SYNC0 on slave
    int sync1Time = 0;
    ec_dcsync01(m_slave_nr, active, sync0Time,sync1Time ,sync0Shift);// both sync0 and sync1 has to be configured
                                             // if this one is not configured the c0251 error appears
}

bool EcSlaveTrack::writeMDT (EcPDOEntry entry, int value) throw(EcErrorSGDV)
{
   if(entry < 0 || entry >= outputObjects.size())
    {
        std::cout<<"entry "<<entry<<std::endl;
       	std::cout<<"outputsize "<<outputObjects.size()<<std::endl;
       	throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    }
    //write on the desired position of the PDO
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[entry].offset, &value ,outputObjects[entry].byteSize);
    slaveOutMutex.unlock();




}

bool EcSlaveTrack::readAT (EcPDOEntry entry, int& value)
{

    if(entry<0 || entry>=inputObjects.size())
    throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    //read the desired position of the PDO
    slaveInMutex.lock();
    memcpy (&value, inputBuf + inputObjects[entry].offset, inputObjects[entry].byteSize);
    //memcpy (&value, ec_slave[1].inputs + inputObjects[entry].offset, inputObjects[entry].byteSize);
    slaveInMutex.unlock();

}

bool EcSlaveTrack::writeControlWord (uint16_t controlWord)
{
    if (!wControlWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[controlWordEntry].offset, &controlWord ,outputObjects[controlWordEntry].byteSize);
    slaveOutMutex.unlock();
}
	
bool EcSlaveTrack::readStatusWord (uint16_t &statusWord)
{
    if (!rStatusWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&statusWord ,inputBuf + inputObjects[statusWordEntry].offset, inputObjects[statusWordEntry].byteSize);
    slaveInMutex.unlock();
    
} 
	
bool EcSlaveTrack::writePosition (int32_t position)
{
    if (!wPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetPositionEntry].offset, &position ,outputObjects[targetPositionEntry].byteSize);
    slaveOutMutex.unlock();
}
	
bool EcSlaveTrack::readPosition (int32_t &position)
{
    if (!rPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&position ,inputBuf + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    slaveInMutex.unlock();
}
	
bool EcSlaveTrack::writeVelocity (int32_t velocity)
{
    if (!wVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetVelocityEntry].offset, &velocity ,outputObjects[targetVelocityEntry].byteSize);
    slaveOutMutex.unlock();
}
	
bool EcSlaveTrack::readVelocity (int32_t &velocity)
{
    if (!rVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&velocity ,inputBuf + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    slaveInMutex.unlock();
}

bool EcSlaveTrack::writeTorque (int16_t torque)
{
    if (!wTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetTorqueEntry].offset, &torque ,outputObjects[targetTorqueEntry].byteSize);
    slaveOutMutex.unlock();
}

bool EcSlaveTrack::readTorque (int16_t &torque)
{
    if (!rTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&torque ,inputBuf + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    slaveInMutex.unlock();
}

void EcSlaveTrack::readXML() throw(EcErrorSGDV)
{
  SoEparameter temp;
	
  //Lectura del xml
  std::string xml_name = "configure_CARRIL.xml";
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(xml_name.c_str());
  
  //Detecion de error 
  if (!result)
  throw(EcErrorSGDV(EcErrorSGDV::XML_NOT_FOUND_ERROR,m_slave_nr,getName()));  
  
  //Acceso a los datos del docmento xml
  pugi::xml_node parameters = doc.first_child();
  
  //Para cada boque <structure> del xml
  for (pugi::xml_node structure = parameters.first_child(); structure; structure = structure.next_sibling())
  {
	//Para cada bloque <param>
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
		else if (name == "idn")
		{
			if (type != "integer")
			throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));	  
			temp.idn = (int16_t) strtol (param.child_value(),NULL,0);          
		}
		else if (name ==  "elementflags")
		{
			if (type != "integer")
			throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
			temp.elementflags = (int8_t) strtol (param.child_value(),NULL,0);
		}
		else if (name ==  "size")
		{
			if (type != "integer")
			throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));	  
			temp.size = (int8_t) strtol (param.child_value(),NULL,0);
		}
        else if (name ==  "editability")
        {
        }
		else if (name ==  "value")
		{
			if (type != "integer")
			throw(EcErrorSGDV(EcErrorSGDV::XML_TYPE_ERROR,m_slave_nr,getName()));
			temp.param = strtol (param.child_value(),NULL,0);
            //addPDOobject(PDOentry,temp.param,temp.subindex);
		}
		else
			throw(EcErrorSGDV(EcErrorSGDV::XML_STRUCTURE_ERROR,m_slave_nr,getName()));
		}
    m_params.push_back(temp);
  }
/*  if(outputObjects.size() <= 0)
      throw(EcErrorSGDV(EcErrorSGDV::OUTPUT_OBJECTS_ERROR,m_slave_nr,getName()));
  if(inputObjects.size() <= 0)
      throw(EcErrorSGDV(EcErrorSGDV::INPUT_OBJECTS_ERROR,m_slave_nr,getName()));
*/
}
	
//No lo acabo de ver
/*
bool EcSlaveTrack::addPDOobject (std::string PDOentry, int value, int subindex)
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
*/
    
void EcSlaveTrack::setSGDVOject(uint16_t idn, uint8_t elementflags, int psize, void * param)
{
	//ec_SDOwrite(m_slave_nr, index, subindex, FALSE,psize,param,EC_TIMEOUTRXM);
    ec_SoEwrite( m_slave_nr, 0, elementflags, idn, psize, param, EC_TIMEOUTRXM);

}

void EcSlaveTrack::getSGDVObject(uint16_t idn, uint8_t elementflags, int *psize, void *param)
{
	//ec_SDOread(m_slave_nr, index, subindex, FALSE,psize,param,EC_TIMEOUTRXM);
    ec_SoEread(m_slave_nr, 0, elementflags, idn, psize, param,EC_TIMEOUTRXM);
}

std::vector<int> EcSlaveTrack::readIDNofIDNs(int idn, bool print)
{

    int idnsize=30;
    char *list2 = new char[idnsize];
    ec_SoEread(1,0, 0x40, idn, &idnsize, list2, EC_TIMEOUTRXM);
    int i;
    std::vector<int> idnlist;
    idnlist.resize(idnsize/2);
    int16 value=0;

    for(i=0;i<idnlist.size();i++)
    {
        memcpy( &value,list2+2*i,2);
        //std::cout<<idn<<std::endl;
        idnlist[i] = value;
    }

    if (print)
    {
        for (i=0;i<idnlist.size();i++)
            std::cout<<idnlist[i]<<std::endl;

    }

    delete[] list2;
    return idnlist;

}

void EcSlaveTrack::setMDT(std::vector<int> listidn)
{
    int listSize=4+2*listidn.size();
    //int listSize = 4+2*nidn;
    char *list = new char[listSize];
    int idn = 2*listidn.size();
    memcpy(list, &idn,2);
    idn = 30;
    memcpy(list+2, &idn,2);
    int i;
    for (i=0;i<listidn.size();i++)
    {
        idn = listidn[i];
        memcpy(list+4+2*i, &idn,2);
    }
    ec_SoEwrite(1, 0, 0x40, 24, listSize, list,EC_TIMEOUTRXM);
    delete[] list;
}

void EcSlaveTrack::setAT(std::vector<int> listidn)
{
    int listSize=4+2*listidn.size();
    //int listSize = 4+2*nidn;
    char *list = new char[listSize];
    int idn = 2*listidn.size();
    memcpy(list, &idn,2);
    idn = 30;
    memcpy(list+2, &idn,2);
    int i;
    for (i=0;i<listidn.size();i++)
    {
        idn = listidn[i];
        memcpy(list+4+2*i, &idn,2);
    }
    ec_SoEwrite(1, 0, 0x40, 16, listSize, list,EC_TIMEOUTRXM);
    delete[] list;
}

void EcSlaveTrack::modeSetUp(EcMode mode, int nmode)
{
    ec_SoEwrite(1, 0, 0x40, 32+nmode, 2, &mode,EC_TIMEOUTRXM);
}

void EcSlaveTrack::TelegramType(EcTelegramType type)
{
    ec_SoEwrite(1, 0, 0x40, 15, 2, &type,EC_TIMEOUTRXM);
}

void EcSlaveTrack::ClearError()
{
    //clear the errors of SlaveTrack
    int clear = 3;
    ec_SoEwrite(1, 0, 0x40, 99, 2, &clear,EC_TIMEOUTRXM);
    clear = 0;
    ec_SoEwrite(1, 0, 0x40, 99, 2, &clear,EC_TIMEOUTRXM);
    std::cout<<"Master configured!!!"<<std::endl;
    usleep(200000);
}

namespace {
cpp4ec::EcSlave* createEcSlaveTrack(ec_slavet* mem_loc)
{
    return new EcSlaveTrack(mem_loc);
}

const bool registered0 = cpp4ec::EcSlaveFactory::Instance().registerDriver("? M:00000024 I:00242803", createEcSlaveTrack);

}
}

