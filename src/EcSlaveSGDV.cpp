#include "EcSlaveSGDV.h"
#include "EcUtil.h"

#include "EcSlaveSGDV.h"
#include "EcSlaveFactory.h"

#include <pugixml.hpp>

#define timestampSize 8


namespace cpp4ec
{

EcSlaveSGDV::EcSlaveSGDV (ec_slavet* mem_loc) : EcSlave (mem_loc),
    outputSize(0), inputSize(0), pBufferOut(NULL),pBufferIn(NULL),inputBuf(NULL),
    controlWordEntry(0), targetPositionEntry(0), targetVelocityEntry(0), targetTorqueEntry(0),
    statusWordEntry(0), actualPositionEntry(0), actualVelocityEntry(0), actualTorqueEntry(0),
    wControlWordCapable(false), wPositionCapable(false), wVelocityCapable(false),wTorqueCapable(false),
    rStatusWordCapable(false), rPositionCapable(false), rVelocityCapable(false), rTorqueCapable(false),
    parameterSetting(false), PDOmapping(false)
    
{
   m_params.resize(0);
   inputObjects.resize(0);
   outputObjects.resize(0);
   m_name = "SGDV_" + to_string(m_datap->configadr & 0x0f,std::dec);  
   
   
   bool xml = readXML();

   if(!xml | !parameterSetting)
       loadParameters();
   
   if(!xml | !PDOmapping)
       loadDefaultPDO();

}
EcSlaveSGDV::~EcSlaveSGDV()
{
    delete[] inputBuf;    
}
    
void EcSlaveSGDV::update()
{
    slaveInMutex.lock();
    memcpy(inputBuf,pBufferIn, inputSize);
    slaveInMutex.unlock();

    unsigned long time;    
    uint16_t statusWord =0;
    int32_t position = 0;
    int32_t velocity = 0;
    int16_t torque = 0;
    readTimestamp(time);
    if(rStatusWordCapable)
	readStatusWord(statusWord);
    if(rPositionCapable)
	readPosition (position);
    if(rVelocityCapable)
	readVelocity (velocity);
    if(rTorqueCapable)
	readTorque (torque);
    //signal
    slaveValues(m_slave_nr,statusWord,position,velocity,torque,time);    
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
    si_PDOassign(m_slave_nr,0x1C12);//sync manager for outputs
    si_PDOassign(m_slave_nr,0x1C13);//sync manager for inputs
    enableSpecificFunctions();

    outputSize = outputObjects[outputObjects.size()-1].offset + outputObjects[outputObjects.size()-1].byteSize;
#ifdef RTNET
    inputSize  = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize + timestampSize;
#else
    inputSize  = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize;
    setPDOBuffer(NULL, NULL);
#endif

    inputBuf = new char[inputSize];
    memset(inputBuf,0, inputSize);


    std::cout << getName() << " configured !" <<std::endl;
    
    return true;
}

void EcSlaveSGDV::start() throw(EcErrorSGDV)
{

  writeControlWord(CW_SHUTDOWN);
#ifdef RTNET
  updateMaster();
#endif
  usleep(100000);
  
  writeControlWord(CW_SWITCH_ON);
#ifdef RTNET
  updateMaster();
#endif
  usleep(100000);
  
  // Enable movement
  writeControlWord(CW_ENABLE_OP);
#ifdef RTNET
  updateMaster();
#endif
  usleep(100000);
  
}

void EcSlaveSGDV::setDC(bool active, unsigned int sync0Time, unsigned int sync0Shift) throw(EcErrorSGDV)
{
    ec_dcsync0(m_slave_nr, active, sync0Time, sync0Shift);
}

void EcSlaveSGDV::setPDOBuffer(char * input, char * output)
{
#ifdef RTNET
    pBufferIn  = input;
    pBufferOut = output;
#else
    pBufferIn  = m_datap -> inputs;
    pBufferOut = m_datap -> outputs;
#endif
}

void EcSlaveSGDV::stop() throw(EcErrorSGDV)
{

  writeControlWord(CW_SHUTDOWN);
#ifdef RTNET
  updateMaster();
#endif
  usleep(100000);
    
  writeControlWord(CW_QUICK_STOP);
#ifdef RTNET
  updateMaster();
#endif
  usleep(100000);
}

bool EcSlaveSGDV::readTimestamp (unsigned long& time)
{
    slaveInMutex.lock();
    memcpy (&time, inputBuf + inputSize - timestampSize, timestampSize);
    slaveInMutex.unlock();
}
    

bool EcSlaveSGDV::writePDO (EcPDOEntry entry, int value)
{
    if(entry < 0 || entry >= outputObjects.size())
	throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    
    //write on the desired position of the PDO
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[entry].offset, &value ,outputObjects[entry].byteSize);
    slaveOutMutex.unlock();
}

bool EcSlaveSGDV::readPDO (EcPDOEntry entry, int& value)
{
    if(entry<0 || entry>=inputObjects.size())
	throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    //read the desired position of the PDO
    slaveInMutex.lock();
    memcpy (&value, inputBuf + inputObjects[entry].offset, inputObjects[entry].byteSize);
    slaveInMutex.unlock();
}

bool EcSlaveSGDV::writeControlWord (uint16_t controlWord)
{
    if (!wControlWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[controlWordEntry].offset, &controlWord ,outputObjects[controlWordEntry].byteSize);
    slaveOutMutex.unlock();

}

bool EcSlaveSGDV::readStatusWord (uint16_t &statusWord)
{
    if (!rStatusWordCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&statusWord ,inputBuf + inputObjects[statusWordEntry].offset, inputObjects[statusWordEntry].byteSize);
    slaveInMutex.unlock();    
} 

bool EcSlaveSGDV::writePosition (int32_t position)
{
    if (!wPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetPositionEntry].offset, &position ,outputObjects[targetPositionEntry].byteSize);
    slaveOutMutex.unlock();

}

bool EcSlaveSGDV::readPosition (int32_t &position)
{
    if (!rPositionCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&position ,inputBuf + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    slaveInMutex.unlock();
}

bool EcSlaveSGDV::writeVelocity (int32_t velocity)
{
    if (!wVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetVelocityEntry].offset, &velocity ,outputObjects[targetVelocityEntry].byteSize);
    slaveOutMutex.unlock();
}

bool EcSlaveSGDV::readVelocity (int32_t &velocity)
{
    if (!rVelocityCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&velocity ,inputBuf + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    slaveInMutex.unlock();
}
bool EcSlaveSGDV::writeTorque (int16_t torque)
{
    if (!wTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetTorqueEntry].offset, &torque ,outputObjects[targetTorqueEntry].byteSize);
    slaveOutMutex.unlock();
}

bool EcSlaveSGDV::readTorque (int16_t &torque)
{
    if (!rTorqueCapable)
	throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
    slaveInMutex.lock();
    memcpy (&torque ,inputBuf + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    slaveInMutex.unlock();
}

bool EcSlaveSGDV::readXML() throw(EcErrorSGDV)
{
  CoEparameter temp;
  std::string xml_name = "configure_SGDV_"+to_string(m_slave_nr,std::dec)+".xml";
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(xml_name.c_str());
  if (!result)
    return false;  
  
  pugi::xml_node configuration = doc.first_child();
  for (pugi::xml_node type = configuration.first_child(); type; type = type.next_sibling())
  {
      std::string typeName = type.name();
      if( typeName == "parameterSetting")
      {
	  parameterSetting = true;
	  for(pugi::xml_node parameter = type.first_child(); parameter; parameter = parameter.next_sibling())
	  {
	      for(pugi::xml_node param = parameter.first_child(); param; param = param.next_sibling())
	      {
	      
		  std::string name = param.name();
		  if (name ==  "description")
		  {
		      temp.description = param.child_value();
		  }else if (name ==  "name")
		  {
		      temp.name = param.child_value();
		  }
		  else if (name == "index")
		  {
			temp.index = (int16_t) strtol (param.child_value(),NULL,0);          
		  }
		  else if (name ==  "subindex")
		  {
		      temp.subindex = (int8_t) strtol (param.child_value(),NULL,0);
		  }
		  else if (name ==  "size")
		  {
		      temp.size = (int8_t) strtol (param.child_value(),NULL,0);
		  }
		  else if (name ==  "value")
		  {
		      temp.param = strtol (param.child_value(),NULL,0);
		  }else{
		      throw(EcErrorSGDV(EcErrorSGDV::XML_STRUCTURE_ERROR,m_slave_nr,getName()));
		  }
	      }
	      m_params.push_back(temp);
      }
      }else if(typeName == "PDOmapping")
      {
	  PDOmapping = true;
	  for(pugi::xml_node parameter = type.first_child(); parameter; parameter = parameter.next_sibling())
	  {
	      for(pugi::xml_node param = parameter.first_child(); param; param = param.next_sibling())
	      {
		  std::string name = param.name();
		  if (name ==  "description")
		  {
		      temp.description = param.child_value();
		  }else if (name ==  "name")
		  {
		      temp.name = param.child_value();
		  }
		  else if (name == "index")
		  {
			temp.index = (int16_t) strtol (param.child_value(),NULL,0);          
		  }
		  else if (name ==  "subindex")
		  {
		      temp.subindex = (int8_t) strtol (param.child_value(),NULL,0);
		  }
		  else if (name ==  "size")
		  {
		      temp.size = (int8_t) strtol (param.child_value(),NULL,0);
		  }
		  else if (name ==  "value")
		  {
                      temp.param = strtol (param.child_value(),NULL,0);
		  }else{
		      throw(EcErrorSGDV(EcErrorSGDV::XML_STRUCTURE_ERROR,m_slave_nr,getName()));
		  }
	      }
	      m_params.push_back(temp);
      }

      }else{
	  throw(EcErrorSGDV(EcErrorSGDV::XML_STRUCTURE_ERROR,m_slave_nr,getName()));
      }
      
  }
  
  return true;
  


}

bool EcSlaveSGDV::enableSpecificFunctions ()
{
    if(outputObjects.size() <= 0)
      throw(EcErrorSGDV(EcErrorSGDV::OUTPUT_OBJECTS_ERROR,m_slave_nr,getName()));
    if(inputObjects.size() <= 0)
      throw(EcErrorSGDV(EcErrorSGDV::INPUT_OBJECTS_ERROR,m_slave_nr,getName()));

    for( int i = 0; i < inputObjects.size();i++)
    {
        switch (inputObjects[i].index)
        {
            case STATUS_WORD:
            statusWordEntry = i;
            rStatusWordCapable = true;
            //std::cout<<"STATUS_WORD "<<i<<std::endl;

            break;

            case ACTUAL_POSITION:
            actualPositionEntry = i;
            rPositionCapable = true;
            //std::cout<<"ACTUAL_POSITION "<<i<<std::endl;
            break;

            case ACTUAL_VELOCITY:
            actualVelocityEntry = i;
            rVelocityCapable = true;
            //std::cout<<"ACTUAL_VELOCITY "<<i<<std::endl;
            break;

            case ACTUAL_TORQUE:
            actualTorqueEntry = i;
            rTorqueCapable = true;
            //std::cout<<"ACTUAL_TORQUE "<<i<<std::endl;
            break;

            default:
            break;
        }
    }

    for( int i = 0; i < outputObjects.size();i++)
    {
        switch (outputObjects[i].index)
        {
            case CONTROL_WORD:
            controlWordEntry = i;
            wControlWordCapable = true;
            //std::cout<<"CONTROL_WORD "<<i<<std::endl;

            break;

            case TARGET_POSITION:
            targetPositionEntry = i;
            wPositionCapable = true;
            //std::cout<<"TARGET_POSITION "<<i<<std::endl;

            break;

            case TARGET_VELOCITY:
            targetVelocityEntry = i;
            wVelocityCapable = true;
            //std::cout<<"TARGET_VELOCITY "<<i<<std::endl;

            break;

            case TARGET_TORQUE:
            targetTorqueEntry = i;
            wTorqueCapable = true;
            //std::cout<<"TARGET_TORQUE "<<i<<std::endl;

            break;

            default:
            break;

        }
    }

    return true;
	
}

void EcSlaveSGDV::loadDefaultPDO()
{
   CoEparameter temp;
   
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
   temp.index = 0x1600;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 0;
   m_params.push_back(temp);
   
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1600;
   temp.subindex = 0x01;
   temp.name = "Control Word ";
   temp.size = 4;
   temp.param = 0x60400010;
   m_params.push_back(temp);
  
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1600;
   temp.subindex = 0x02;
   temp.name = "Target Position ";
   temp.size = 4;
   temp.param = 0x607A0020;
   m_params.push_back(temp);

   temp.description = "Receive PDO Mapping";
   temp.index = 0x1600;
   temp.subindex = 0x03;
   temp.name = "Target Velovity ";
   temp.size = 4;
   temp.param = 0x60FF0020;
   m_params.push_back(temp);

   temp.description = "Receive PDO Mapping";
   temp.index = 0x1600; 
   temp.subindex = 0x04;
   temp.name = "Target Torque ";
   temp.size = 4; 
   temp.param = 0x60710010;
   m_params.push_back(temp);
                     
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1600; 
   temp.subindex = 0x05;
   temp.name = "Mode of Operation ";
   temp.size = 4; 
   temp.param = 0x60600008;
   m_params.push_back(temp);
                                             
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 0;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00;
   temp.subindex = 0x01;
   temp.name = "Status Word";
   temp.size = 4;
   temp.param = 0x60410010;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00;
   temp.subindex = 0x02;
   temp.name = "Position Actual Value";
   temp.size = 4;
   temp.param = 0x60640020;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00;
   temp.subindex = 0x03;
   temp.name = "Velocity Actual Value";
   temp.size = 4;
   temp.param = 0x606C0020;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00; 
   temp.subindex = 0x04;
   temp.name = "Torque Actual Value";
   temp.size = 4; 
   temp.param = 0x60770010;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00; 
   temp.subindex = 0x05;
   temp.name = "Error Code";
   temp.size = 4; 
   temp.param = 0x603F0010;
   m_params.push_back(temp);
                                  
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00; 
   temp.subindex = 0x06;
   temp.name = "Mode of Operation Display";
   temp.size = 4; 
   temp.param = 0x60610008;
   m_params.push_back(temp);
   
   //3.Set the number of mapping entries in PDO mapping objects 
   temp.description = "Receive PDO Mapping";
   temp.index = 0x1600;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 5;
   m_params.push_back(temp);
   
   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00;
   temp.subindex = 0x00;
   temp.name = "Number of objects";
   temp.size = 1;
   temp.param = 6;
   m_params.push_back(temp);

   //4.Set the assignment of the Sync manager and PDO    
   temp.description = "Assing";
   temp.index = 0x1C12;
   temp.subindex = 0x01;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 2;
   temp.param = 0x1600;
   m_params.push_back(temp);
   
   temp.description = "Assing";
   temp.index = 0x1C13;
   temp.subindex = 0x01;
   temp.name = "Sync Manager PDO Assignment";
   temp.size = 2;
   temp.param = 0x1A00;
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

void EcSlaveSGDV::loadParameters()
{
   CoEparameter temp;
   //setting parameters 
   temp.description = "Modes of Operation";
   temp.index = 0x6060;
   temp.subindex = 0x00;
   temp.name = "OpMode";
   temp.size = 1;
   temp.param = 1;
   m_params.push_back(temp);
   
   temp.description = "Profile velocity [mdeg/s]";
   temp.index = 0x6081;
   temp.subindex = 0x00;
   temp.name = "Pvel";
   temp.size = 4;
   temp.param = 150000;
   m_params.push_back(temp);
   
   temp.description = "Max. Profile velocity [mdeg/s]";
   temp.index = 0x607F;
   temp.subindex = 0x00;
   temp.name = "MaxVel";
   temp.size = 4;
   temp.param = 200000;
   m_params.push_back(temp);

   temp.description = "Profile acceleration";
   temp.index = 0x6083;
   temp.subindex = 0x00;
   temp.name = "Pacc";
   temp.size = 4;
   temp.param = 150;
   m_params.push_back(temp);
                       
   temp.description = "Profile deceleration";
   temp.index = 0x6084;
   temp.subindex = 0x00;
   temp.name = "Pdec";
   temp.size = 4;
   temp.param = 150;
   m_params.push_back(temp);
   
   //Torque parameters (mode = 4)
   temp.description = "Max Torque";
   temp.index = 0x6072;
   temp.subindex = 0x00;
   temp.name = "MaxTorq";
   temp.size = 2;
   temp.param = 1000;
   m_params.push_back(temp);
   
   temp.description = "Torque Slope";
   temp.index = 0x6087;
   temp.subindex = 0x00;
   temp.name = "TSlope";
   temp.size = 4;
   temp.param = 100;
   m_params.push_back(temp);
}
    
void EcSlaveSGDV::setSGDVObject(uint16_t index, uint8_t subindex, int psize, void * param)
{
  ec_SDOwrite(m_slave_nr, index, subindex, FALSE,psize,param,EC_TIMEOUTRXM);
}

void EcSlaveSGDV::getSGDVObject(uint16_t index, uint8_t subindex, int *psize, void *param)
{
  ec_SDOread(m_slave_nr, index, subindex, FALSE,psize,param,EC_TIMEOUTRXM);
}

void EcSlaveSGDV::si_PDOassign(uint16 slave, uint16 PDOassign)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;
    uint8 bitlen, obj_subidx;
    uint16 obj_idx;

    rdl = sizeof(rdat); rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    bool outputMapping = false, inputMapping = false;
    if(PDOassign == 0x1C12)//sync manager for outputs
        outputMapping = true;
    if(PDOassign == 0x1C13)//sync manager for inputs
        inputMapping = true;

    /* positive result from slave ? */
    if ((wkc > 0) && (rdat > 0))
    {
    /* number of available sub indexes */
    nidx = rdat;
    bsize = 0;
    /* read all PDO's */
    for (idxloop = 1; idxloop <= nidx; idxloop++)
    {
        rdl = sizeof(rdat); rdat = 0;
        /* read PDO assign */
        wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
        /* result is index of PDO */
        idx = etohl(rdat);
        if (idx > 0)
        {
        rdl = sizeof(subcnt); subcnt = 0;
        /* read number of subindexes of PDO */
        wkc = ec_SDOread(slave,idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
        subidx = subcnt;
        /* for each subindex */
        for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
        {
            PDOobject temp;

            rdl = sizeof(rdat2); rdat2 = 0;
            /* read SDO that is mapped in PDO */
            wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
            rdat2 = etohl(rdat2);
            /* extract bitlength of SDO */
            bitlen = LO_BYTE(rdat2);
            bsize += bitlen;
            obj_idx = (uint16)(rdat2 >> 16);
            obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
            temp.index = obj_idx;
            temp.byteSize = bitlen/8;
            temp.entry = subidxloop;

            if(outputMapping)
            {
                if(outputObjects.size()==0)
                    temp.offset = 0;
                else
                    temp.offset = outputObjects[outputObjects.size()-1].offset + outputObjects[outputObjects.size()-1].byteSize;
                outputObjects.push_back(temp);

            }
            if(inputMapping)
            {
                if(inputObjects.size()==0)
                    temp.offset = 0;
                else
                    temp.offset = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize;
                inputObjects.push_back(temp);

            }
        };
        };
    };
    };
}
namespace {
cpp4ec::EcSlave* createEcSlaveSGDV(ec_slavet* mem_loc)
{
	return new EcSlaveSGDV(mem_loc);
}

const bool registered0 = cpp4ec::EcSlaveFactory::Instance().registerDriver("? M:00000539 I:02200001", createEcSlaveSGDV);

}
}

