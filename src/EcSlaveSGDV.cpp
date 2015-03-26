#include "EcSlaveSGDV.h"
#include "EcUtil.h"

#include "EcSlaveSGDV.h"
#include "EcSlaveFactory.h"

#include <pugixml.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#ifdef RT
#include <rtdk.h> //rt print header
#endif

#define timestampSize 8

#ifdef RT
extern int64 EcTimeStamp;
#endif

namespace cpp4ec
{
#ifndef RTNET
    extern std::mutex slaveOutMutex;
#endif
#ifdef RT
    RT_MUTEX mutex;
#endif

EcSlaveSGDV::EcSlaveSGDV (ec_slavet* mem_loc) : EcSlave (mem_loc),
    controlWordEntry(0), targetPositionEntry(0), targetVelocityEntry(0), targetTorqueEntry(0),
    statusWordEntry(0), actualPositionEntry(0), actualVelocityEntry(0), actualTorqueEntry(0),
    wControlWordCapable(false), wPositionCapable(false), wVelocityCapable(false),wTorqueCapable(false),
    rStatusWordCapable(false), rPositionCapable(false), rVelocityCapable(false), rTorqueCapable(false),
    parameterSetting(false), PDOmapping(false), inputShift(125000)

#if defined(HRT) || !defined(RTNET)
    ,outputSize(0), inputSize(0), pBufferOut(NULL),pBufferIn(NULL),inputBuf(NULL)
#endif

{
   m_params.resize(0);
   inputObjects.resize(0);
   outputObjects.resize(0);
   m_name = "SGDV_" + to_string(m_datap->configadr & 0x0f,std::dec);
   
   bool xml = readXML();

   //When there are no parameterSetting structure in the xml the default parameters are loaded
   if(!xml | !parameterSetting)
       loadParameters();

   //When there are no PDOmapping structure in the xml the default PDO is loaded
   if(!xml | !PDOmapping)
       loadDefaultPDO();

}
EcSlaveSGDV::~EcSlaveSGDV()
{
#if defined(HRT) || !defined(RTNET)
    delete[] inputBuf;    
#endif
}
    
void EcSlaveSGDV::update()
{
#if defined(HRT) || !defined(RTNET)
    //Copy the slave data from the master buffer to slave buffer
    slaveInMutex.lock();
    memcpy(inputBuf,pBufferIn, inputSize);
    slaveInMutex.unlock();    

    ActualValue value;
    readActualValue(value);

    uint16_t statusWord =0;
    if(rStatusWordCapable)
        readStatusWord(statusWord);

    //After reading the important slave information a signal is sent
    slaveValues(m_slave_nr,statusWord,value.position,value.velocity,value.torque,value.timestamp);
#endif
}

const std::string& EcSlaveSGDV::getName() const
{
    return m_name;
}

bool EcSlaveSGDV::configure() throw(EcErrorSGDV)
{
    //Parameter are configured using SDO (acyclic communication)
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
#ifdef HRT
    inputSize  = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize + timestampSize;
#endif
#ifndef RTNET
    inputSize  = inputObjects[inputObjects.size()-1].offset + inputObjects[inputObjects.size()-1].byteSize;
    setPDOBuffer(NULL, NULL);
#endif
#if defined(HRT) || !defined(RTNET)
    inputBuf = new char[inputSize];
    memset(inputBuf,0, inputSize);
#endif
#if RT
    rt_printf("%s configured ! \n",m_name.c_str());
#else
    std::cout<<m_name<<" configured !"<<std::endl;
#endif
    
    return true;
}

void EcSlaveSGDV::start() throw(EcErrorSGDV)
{
  /*The state Machine is controled throught controlWord.
   *So the following sequence is necessary to enable
   *the motor movement.
   */
  writeControlWord(CW_SHUTDOWN);
#ifdef HRT
  updateMaster();
#endif
  usleep(100000);
  
  writeControlWord(CW_SWITCH_ON);
#ifdef HRT
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
    //Configure Sync0 event and input Shift respect this event
    setSGDVObject(0x1C33,0x03,4,&inputShift);
    ec_dcsync0(m_slave_nr, active, sync0Time, sync0Shift);
}

void EcSlaveSGDV::setPDOBuffer(char * input, char * output)
{
#ifdef HRT
    pBufferIn  = input;
    pBufferOut = output;
#else
    pBufferIn  = m_datap -> inputs;
    pBufferOut = m_datap -> outputs;
#endif
}

void EcSlaveSGDV::stop() throw(EcErrorSGDV)
{
 //Taking state machine to inital state.
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

bool EcSlaveSGDV::readTimestamp (int64 &time)
{
#ifdef HRT
    slaveInMutex.lock();
    memcpy (&time, inputBuf + inputSize - timestampSize, timestampSize);
    slaveInMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    time = EcTimeStamp;
    rt_mutex_release (&mutex);
#endif
}
    

bool EcSlaveSGDV::writePDO (EcPDOEntry entry, int value)
{
    //If the PDO have less entries from the specified one (entry) an error is throwed
    if(entry < 0 || entry >= outputObjects.size())
        throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));

#if defined(HRT) || !defined(RTNET)
    //write on the desired position of the PDO
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[entry].offset, &value ,outputObjects[entry].byteSize);
    slaveOutMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (m_datap -> outputs + outputObjects[entry].offset, &value ,outputObjects[entry].byteSize);
    rt_mutex_release (&mutex);
#endif

}

bool EcSlaveSGDV::readPDO (EcPDOEntry entry, int& value)
{
    if(entry<0 || entry>=inputObjects.size())
        throw(EcErrorSGDV(EcErrorSGDV::WRONG_ENTRY_ERROR,m_slave_nr,getName()));
    //read the desired position of the PDO
#if defined(HRT) || !defined(RTNET)
    slaveInMutex.lock();
    memcpy (&value, inputBuf + inputObjects[entry].offset, inputObjects[entry].byteSize);
    slaveInMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (&value, m_datap -> inputs + inputObjects[entry].offset, inputObjects[entry].byteSize);
    rt_mutex_release (&mutex);
#endif

}

/*
 * The position of the specific parameters is obtained using the xml
 * or the know default loaded parameters.
 * */
bool EcSlaveSGDV::readActualValue (ActualValue &value)
{
    //If the PDO doesn't have mapped one of the essential parameters, an error is throwed
    if (!rPositionCapable || !rVelocityCapable ||!rTorqueCapable )
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveInMutex.lock();
    memcpy (&(value.position) ,inputBuf + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    memcpy (&(value.velocity) ,inputBuf + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    memcpy (&(value.torque) ,inputBuf + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    memcpy (&(value.timestamp), inputBuf + inputSize - timestampSize, timestampSize);
    slaveInMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (&(value.position) ,m_datap -> inputs + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    memcpy (&(value.velocity) ,m_datap -> inputs + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    memcpy (&(value.torque) ,m_datap -> inputs + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    value.timestamp= EcTimeStamp;
    rt_mutex_release (&mutex);
#endif


}

bool EcSlaveSGDV::writeControlWord (uint16_t controlWord)
{
    if (!wControlWordCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[controlWordEntry].offset, &controlWord ,outputObjects[controlWordEntry].byteSize);
    slaveOutMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (m_datap -> outputs + outputObjects[controlWordEntry].offset, &controlWord ,outputObjects[controlWordEntry].byteSize);
    rt_mutex_release (&mutex);
#endif

}

bool EcSlaveSGDV::readStatusWord (uint16_t &statusWord)
{
    if (!rStatusWordCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveInMutex.lock();
    memcpy (&statusWord ,inputBuf + inputObjects[statusWordEntry].offset, inputObjects[statusWordEntry].byteSize);
    slaveInMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (&statusWord ,m_datap -> inputs + inputObjects[statusWordEntry].offset, inputObjects[statusWordEntry].byteSize);
    rt_mutex_release (&mutex);
#endif
} 

bool EcSlaveSGDV::writePosition (int32_t position)
{
    if (!wPositionCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetPositionEntry].offset, &position ,outputObjects[targetPositionEntry].byteSize);
    slaveOutMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (m_datap -> outputs + outputObjects[targetPositionEntry].offset, &position ,outputObjects[targetPositionEntry].byteSize);
    rt_mutex_release (&mutex);
#endif

}

bool EcSlaveSGDV::readPosition (int32_t &position)
{
    if (!rPositionCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveInMutex.lock();
    memcpy (&position ,inputBuf + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    slaveInMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (&position ,m_datap -> inputs + inputObjects[actualPositionEntry].offset, inputObjects[actualPositionEntry].byteSize);
    rt_mutex_release (&mutex);
#endif
}

bool EcSlaveSGDV::writeVelocity (int32_t velocity)
{
    if (!wVelocityCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetVelocityEntry].offset, &velocity ,outputObjects[targetVelocityEntry].byteSize);
    slaveOutMutex.unlock();
#endif
#if RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (m_datap -> outputs + outputObjects[targetVelocityEntry].offset, &velocity ,outputObjects[targetVelocityEntry].byteSize);
    rt_mutex_release (&mutex);
#endif
}

bool EcSlaveSGDV::readVelocity (int32_t &velocity)
{
    if (!rVelocityCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveInMutex.lock();
    memcpy (&velocity ,inputBuf + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    slaveInMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (&velocity ,m_datap -> inputs + inputObjects[actualVelocityEntry].offset, inputObjects[actualVelocityEntry].byteSize);
    rt_mutex_release (&mutex);
#endif
}

bool EcSlaveSGDV::writeTorque (int16_t torque)
{
    if (!wTorqueCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveOutMutex.lock();
    memcpy (pBufferOut + outputObjects[targetTorqueEntry].offset, &torque ,outputObjects[targetTorqueEntry].byteSize);
    slaveOutMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (m_datap -> outputs + outputObjects[targetTorqueEntry].offset, &torque ,outputObjects[targetTorqueEntry].byteSize);
    rt_mutex_release (&mutex);
#endif
}

bool EcSlaveSGDV::readTorque (int16_t &torque)
{
    if (!rTorqueCapable)
        throw(EcErrorSGDV(EcErrorSGDV::FUNCTION_NOT_ALLOWED_ERROR,m_slave_nr,getName()));
#if defined(HRT) || !defined(RTNET)
    slaveInMutex.lock();
    memcpy (&torque ,inputBuf + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    slaveInMutex.unlock();
#endif
#ifdef RT
    rt_mutex_acquire (&mutex,TM_INFINITE);
    memcpy (&torque ,m_datap -> inputs + inputObjects[actualTorqueEntry].offset, inputObjects[actualTorqueEntry].byteSize);
    rt_mutex_release (&mutex);
#endif
}

bool EcSlaveSGDV::readXML() throw(EcErrorSGDV)
{
  CoEparameter temp;
  std::string xml_name = "configure_SGDV_"+to_string(m_slave_nr,std::dec)+".xml";
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(xml_name.c_str());
  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;
  std::string home(homedir);

  if (!result)
  {
      xml_name = home+"/cpp4ec_config/configure_SGDV_"+to_string(m_slave_nr,std::dec)+".xml";
      result = doc.load_file(xml_name.c_str());
      if (!result)
          return false;
  }
                         
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
            break;

            case ACTUAL_POSITION:
            actualPositionEntry = i;
            rPositionCapable = true;
            break;

            case ACTUAL_VELOCITY:
            actualVelocityEntry = i;
            rVelocityCapable = true;
            break;

            case ACTUAL_TORQUE:
            actualTorqueEntry = i;
            rTorqueCapable = true;
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
            break;

            case TARGET_POSITION:
            targetPositionEntry = i;
            wPositionCapable = true;
            break;

            case TARGET_VELOCITY:
            targetVelocityEntry = i;
            wVelocityCapable = true;
            break;

            case TARGET_TORQUE:
            targetTorqueEntry = i;
            wTorqueCapable = true;
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

