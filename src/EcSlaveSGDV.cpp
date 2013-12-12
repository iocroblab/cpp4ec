#include "EcSlaveSGDV.h"

#include <sys/mman.h>
#include "EcSlaveSGDV.h"
#include "EcSlaveFactory.h"

//Xenomai
#include <native/task.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>

#include <mxml.h>
#include <stdint.h>





namespace ec4cpp
{

extern RT_MUTEX mutex;

EcSlaveSGDV::EcSlaveSGDV (ec_slavet* mem_loc) : EcSlave (mem_loc),
    useDC (true), SYNC0TIME (1000000), SHIFT (125000),
    SHIFTMASTER (1000000), PDOerrorsTolerance (9)
{
   parameter temp;
   
   readXML();
   
   //setting parameters
//    temp.description = "Modes of Operation";
//    temp.index = 0x6060;
//    temp.subindex = 0x00;
//    temp.name = "OpMode";
//    temp.size = 1;
//    temp.param = 3;
//    m_params.push_back(temp);
   
   //Position parameter (mode = 1)
   temp.description = "Min Position Limit";
   temp.index = 0x607D;
   temp.subindex = 0x01;
   temp.name = "MinPos";
   temp.size = 4;
   temp.param = -80000; 
   m_params.push_back(temp);

   temp.description = "Max Position Limit";
   temp.index = 0x607D;
   temp.subindex = 0x02;
   temp.name = "MaxPos";
   temp.size = 4;
   temp.param = 80000;  
   m_params.push_back(temp);

   temp.description = "Profile velocity";
   temp.index = 0x6081;
   temp.subindex = 0x00;
   temp.name = "Pvel"; 
   temp.size = 4;
   temp.param = 150000;
   m_params.push_back(temp);

   //Velocity Parameters (mode = 3,9)
   
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

   //Torque parameters (mode = 4)
   temp.description = "Max Torque";
   temp.index = 0x6072;
   temp.subindex = 0x00;
   temp.name = "MaxTorq";
   temp.size = 2;
   temp.param = 1000;
   m_params.push_back(temp);

   temp.description = "Slope Torque";
   temp.index = 0x6087;
   temp.subindex = 0x00;
   temp.name = "STorq"; 
   temp.size = 4;
   temp.param = 100;
   m_params.push_back(temp);

   /*-----PDO Mapping-----*/
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

/*   temp.description = "Receive PDO Mapping";
   temp.index = 0x1600; 
   temp.subindex = 0x05;
   temp.name = "Max Torque ";
   temp.size = 4; 
   temp.param = 0x60720010;
   m_params.push_back(temp);
*/                     
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
   temp.subindex = 0x01;
   temp.name = "Position Actual Value";
   temp.size = 4;
   temp.param = 0x60640020;
   m_params.push_back(temp);

   temp.description = "Transmit PDO Mapping";
   temp.index = 0x1A00;
   temp.subindex = 0x02;
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
	throw(EcErrorSGDV(EcErrorSGDV::ECAT_ERROR));

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

void EcSlaveSGDV::update()
{
}

void EcSlaveSGDV::readXML() throw(EcErrorSGDV)
{
  parameter temp; 
  FILE *fp;
   mxml_node_t *tree;
   std::string name = "configure_SGDV_"+to_string(m_slave_nr,std::dec)+".xml";
   const char * cname = name.c_str();
   std::cout<<"Reading "<<cname<<std::endl;
   fp = fopen(cname, "r");
   tree = mxmlLoadFile(NULL, fp, MXML_INTEGER_CALLBACK);//MXML_OPAQUE_CALLBACK might work, lood CDATA
   if(!tree)
     std::cout<<"Error:no xml"<<std::endl;
    // throw(EcErrorSGDV(EcErrorSGDV::XML_NOT_FOUND_ERROR));  

   mxml_node_t *parameters;
   mxml_node_t *structure;
   mxml_node_t *param;
   parameters = mxmlWalkNext(tree, tree, MXML_DESCEND);
   structure = mxmlWalkNext(parameters, tree, MXML_DESCEND);
   
   int i;
   while(structure)
   {
     param = mxmlWalkNext(structure, tree, MXML_DESCEND);
     i=0;     
     while(i < 4)
     {	 
	const char *name;
	name = mxmlElementGetAttr(param, "name");
	
	if (!strcmp(name, "index"))
	{
	  const char *description;
	  description = mxmlElementGetAttr(param, "description");
	  temp.index = (int16_t) param->child->value.integer;
          temp.description = description;
	}
	else if (!strcmp(name, "subindex"))
	  temp.subindex = (int8_t) param->child->value.integer;
	else if (!strcmp(name, "size"))
	  temp.size = (int8_t) param->child->value.integer;
	else if (!strcmp(name, "value"))
	  temp.param = param->child->value.integer;
	else
	  std::cout<<"structure error"<<std::endl;
	 // throw(EcErrorSGDV(EcErrorSGDV::XML_STRUCTURE_ERROR));
	if(i==3)
	 temp.name = "0";

	param = mxmlWalkNext(param, tree, MXML_NO_DESCEND);	 
	i++; 
     }
     m_params.push_back(temp);
     structure = mxmlWalkNext(structure, tree, MXML_NO_DESCEND);

   }    
   fclose(fp);
   mxmlDelete(tree);
}

namespace {
ec4cpp::EcSlave* createEcSlaveSGDV(ec_slavet* mem_loc)
{
	return new EcSlaveSGDV(mem_loc);
}

const bool registered0 = ec4cpp::EcSlaveFactory::Instance().registerDriver("? M:00000539 I:02200001", createEcSlaveSGDV);

}
}

