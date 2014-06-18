#include "EcMaster.h"
#include "EcUtil.h"
extern "C"
{
#include "EcRTThread/EcRTThread.h"
}
#include <sys/mman.h>
#include <iostream>
#include <fstream> //Just to verify the time

//socket header
#include <rtdm/rtipc.h>
#include <sys/types.h>
#include <unistd.h>
//#include <sched.h>
//posix
#include <pthread.h>
//xenomai header
#include <native/task.h>
//boost
#include <boost/thread.hpp>

#define timestampSize 8 //8 bytes to represent the time

namespace cpp4ec
{
   RT_TASK *task;
   RT_TASK master;


EcMaster::EcMaster(std::string ecPort, unsigned long cycleTime, bool useDC, bool slaveInfo) : ethPort (ecPort), m_cycleTime(cycleTime), m_useDC(useDC),
    inputSize(0),outputSize(0), threadFinished (false), slaveInformation(slaveInfo), printSDO(true), printMAP(true),SGDVconnected(false)

{
   //reset del iomap memory
   for (size_t i = 0; i < 4096; i++)
      m_IOmap[i] = 0;

   pid_t pid;
   pid = getpid();
   sched_param param;
   param.sched_priority = 20;
   sched_getscheduler(pid);

/* pthread_t pid =	pthread_self();
   sched_param param;
   param.sched_priority = 20;
   pthread_setscheduler(pid,SCHED_FIFO,&param);
*/
   //Realtime tasks
   mlockall (MCL_CURRENT | MCL_FUTURE);
   rt_task_shadow (&master, "EcMaster",20, T_JOINABLE);
   
}

EcMaster::~EcMaster()
{
   //must clean memory and delete tasks
}

bool EcMaster::preconfigure() throw(EcError)
{
    bool success;
    int size = ethPort.size();
    m_ecPort = new char[size];
    strcpy (m_ecPort, ethPort.c_str());
    task = new RT_TASK;

   
   // initialise SOEM, bind socket to ifname
    if (!(ec_init(m_ecPort) > 0))
        throw(EcError(EcError::FAIL_EC_INIT));
    
    std::cout << "ec_init on " << ethPort << " succeeded." << std::endl;


    if (!(ec_config_init(FALSE) > 0))
        throw(EcError(EcError::FAIL_EC_CONFIG_INIT));

    std::cout << ec_slavecount << " slaves found and configured."<< std::endl;
    std::cout << "Request PRE-OPERATIONAL state for all slaves"<< std::endl;

    success = switchState (EC_STATE_PRE_OP);
    if (!success)
        throw( EcError (EcError::FAIL_SWITCHING_STATE_PRE_OP));
    
    std::cout << "PRE-OPERATIONAL state reached"<< std::endl;
    for (int i = 1; i <= ec_slavecount; i++)
    {
        EcSlave* driver = EcSlaveFactory::Instance().createDriver(&ec_slave[i]);
        if (!driver)
            std::cout<<"Error: Failed creating driver"<<std::endl;
                //throw(EcError(EcError::FAIL_CREATING_DRIVER));

        m_drivers.push_back(driver);
        std::cout << "Created driver for " << ec_slave[i].name<< ", with address " << ec_slave[i].configadr<< std::endl;
        //driver -> configure();
    }
    //The SGDV slave works different from the standard soem sequence
    //so it is detected to modify the structure
    for (int i = 0; i < m_drivers.size(); i++)
    {
        std::string sgdvName = "SGDV";
        std::string slaveName = m_drivers[i]-> getName();
        if(slaveName.compare(0,4,sgdvName) == 0)
            SGDVconnected = true;
    }

    for (int i = 0; i < m_drivers.size(); i++)
        m_drivers[i]-> updateMaster.connect(boost::bind(&EcMaster::update,this));

    std::cout<<"Master preconfigured!!!"<<std::endl;

}

bool EcMaster::configure() throw(EcError)
{
    bool success;
    for (int i = 0; i < m_drivers.size(); i++)
        m_drivers[i] -> configure();

    if(slaveInformation)
        slaveInfo();
    //Configure IOmap
    ec_config_map(&m_IOmap);
    //Configure DC
    if (m_useDC)
    {
        ec_configdc();
        for (int i = 0; i <  m_drivers.size(); i++)
            m_drivers[i] -> setDC(true, m_cycleTime, 0);
    }
    /* The SGDV slave need to activate the cyclic communication
     * just after of activating the distributed clocks
     */
    taskFinished = false;
    if(SGDVconnected)
    {

        rt_task_create (task, "PDO rt_task", 8192, 99, T_JOINABLE);
        rt_task_set_periodic (task, TM_NOW, m_cycleTime);
        rt_task_start (task, &rt_thread, NULL);
    }

    if(EcatError)
	throw(EcError(EcError::ECAT_ERROR));  


    //Create master buffers
    outputSize = ec_slave[0].Obytes;
    inputSize = ec_slave[0].Ibytes + ec_slavecount * timestampSize;

    outputBuf = new char [outputSize];
    inputBuf = new char[inputSize];
    memset(outputBuf,0, outputSize);
    memset(inputBuf,0, inputSize);

    int offSetInput = 0, offSetOutput = 0;
    for(int i = 0; i < m_drivers.size();i++)
    {

        m_drivers[i] -> setPDOBuffer(inputBuf + offSetInput, outputBuf + offSetOutput);
        if(i < (m_drivers.size()-1))
        {
            offSetOutput += ec_slave[i+1].Obytes;
            offSetInput += ec_slave[i+1].Ibytes + timestampSize;

        }
    }

    std::cout << "Request SAFE-OPERATIONAL state for all slaves" << std::endl;
    success = switchState (EC_STATE_SAFE_OP);
    if (!success)
        throw(EcError(EcError::FAIL_SWITCHING_STATE_SAFE_OP));
    
    std::cout << "SAFE-OPERATIONAL state reached"<< std::endl;
    
    if(EcatError)
        throw(EcError(EcError::ECAT_ERROR));


    std::cout << "Request OPERATIONAL state for all slaves" << std::endl;
    success = switchState(EC_STATE_OPERATIONAL);
    if (!success)
	    throw(EcError(EcError::FAIL_SWITCHING_STATE_OPERATIONAL));
    std::cout << "OPERATIONAL state reached" << std::endl;

    if(!SGDVconnected)
    {
        rt_task_create (task, "PDO rt_task", 8192, 99, T_JOINABLE);
        rt_task_set_periodic (task, TM_NOW, m_cycleTime);
        rt_task_start (task, &rt_thread, NULL);
    }
    usleep(200000);

    std::cout<<"Master configured!!!"<<std::endl;

	
    return true;
}


bool EcMaster::start() throw(EcError)
{
   rt_task_set_priority(&master,0);
   
   if (asprintf(&devnameOutput, "/proc/xenomai/registry/rtipc/xddp/%s", XDDP_PORT_OUTPUT) < 0)
       throw(EcError(EcError::FAIL_OUTPUT_LABEL));
   
   fdOutput = open(devnameOutput, O_WRONLY);
   free(devnameOutput);
   
   if (fdOutput < 0)
   {
      perror("open");
      throw(EcError(EcError::FAIL_OPENING_OUTPUT));
   }
   if (asprintf(&devnameInput, "/proc/xenomai/registry/rtipc/xddp/%s", XDDP_PORT_INPUT) < 0)
       throw(EcError(EcError::FAIL_INPUT_LABEL));
              
   fdInput = open(devnameInput, O_RDONLY);
   free(devnameInput);
       
   if (fdInput < 0)
   {
     perror("open");
     throw(EcError(EcError::FAIL_OPENING_INPUT));
   }
   sched_param sch;
   sch.sched_priority = 90;
   pthread_setschedparam(updateThread.native_handle(), SCHED_OTHER, &sch);
   updateThread = std::thread(&EcMaster::update_EcSlaves,this);

   for (int i = 0 ; i < m_drivers.size() ; i++)
       m_drivers[i] ->  start();

   std::cout<<"Master started!!!"<<std::endl;

   return true;
}

/* This function waits the current state from the xddp port */  
void EcMaster::update_EcSlaves(void) throw(EcError)
{
    int ret;
    
    while (!threadFinished) 
    {

        /* Get the next message from realtime_thread */
        ret = read(fdInput, inputBuf, inputSize);
        if (ret <= 0)
        {
            perror("read");
            //throw(EcError(EcError::FAIL_READING));
        }

        for(int i = 0; i < m_drivers.size();i++)
            m_drivers[i] -> update();
	        
    }
}
   
/* This function uses the sockect createdx in the configure to send data to the realtime thread */
void EcMaster::update(void) throw(EcError)
{
   int ret;   
   /* Send a message to realtime_thread */
   ret = write(fdOutput,outputBuf,outputSize);
   if(ret<=0)
   {
       perror("write");
       throw(EcError(EcError::FAIL_WRITING));
   }
}

std::vector<EcSlave*> EcMaster::getSlaves()
{
    return m_drivers;
}

bool EcMaster::stop() throw(EcError)
{


  rt_task_set_priority(&master,20);

  //Stops slaves
  for (int i = 0 ; i < m_drivers.size() ; i++)
      m_drivers[i] ->  stop();

  //Stops the NRT thread
  threadFinished = true; 
  updateThread.join();
  threadFinished = false; 
  

  close(fdInput);
  close(fdOutput);
  std::cout<<"Master stoped!"<<std::endl;
  return true;
}

bool EcMaster::reset() throw(EcError)
{
   if (m_useDC)
   {
      for (int i = 0; i <  m_drivers.size(); i++)
          m_drivers[i] -> setDC(false, m_cycleTime, 0);
   }
   taskFinished = true;
   //Wait on the termination of task. 
   rt_task_join (task);
   // delete the periodic task
   rt_task_delete(task);
   
   bool success = switchState (EC_STATE_INIT);
   if (!success)
      throw(EcError(EcError::FAIL_SWITCHING_STATE_INIT));

   for (unsigned int i = 0; i < m_drivers.size(); i++)
       delete m_drivers[i];
   m_drivers.resize(0);      
   
   delete[] outputBuf;
   delete[] inputBuf;
   delete[] m_ecPort;
   delete task;
   
   
   for (size_t i = 0; i < 4096; i++)
      m_IOmap[i] = 0;

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

char* EcMaster::dtype2string(uint16 dtype)
{
   switch(dtype)
   {
      case ECT_BOOLEAN:
         sprintf(hstr, "BOOLEAN");
         break;
      case ECT_INTEGER8:
         sprintf(hstr, "INTEGER8");
         break;
      case ECT_INTEGER16:
         sprintf(hstr, "INTEGER16");
         break;
      case ECT_INTEGER32:
         sprintf(hstr, "INTEGER32");
         break;
      case ECT_INTEGER24:
         sprintf(hstr, "INTEGER24");
         break;
      case ECT_INTEGER64:
         sprintf(hstr, "INTEGER64");
         break;
      case ECT_UNSIGNED8:
         sprintf(hstr, "UNSIGNED8");
         break;
      case ECT_UNSIGNED16:
         sprintf(hstr, "UNSIGNED16");
         break;
      case ECT_UNSIGNED32:
         sprintf(hstr, "UNSIGNED32");
         break;
      case ECT_UNSIGNED24:
         sprintf(hstr, "UNSIGNED24");
         break;
      case ECT_UNSIGNED64:
         sprintf(hstr, "UNSIGNED64");
         break;
      case ECT_REAL32:
         sprintf(hstr, "REAL32");
         break;
      case ECT_REAL64:
         sprintf(hstr, "REAL64");
         break;
      case ECT_BIT1:
         sprintf(hstr, "BIT1");
         break;
      case ECT_BIT2:
         sprintf(hstr, "BIT2");
         break;
      case ECT_BIT3:
         sprintf(hstr, "BIT3");
         break;
      case ECT_BIT4:
         sprintf(hstr, "BIT4");
         break;
      case ECT_BIT5:
         sprintf(hstr, "BIT5");
         break;
      case ECT_BIT6:
         sprintf(hstr, "BIT6");
         break;
      case ECT_BIT7:
         sprintf(hstr, "BIT7");
         break;
      case ECT_BIT8:
         sprintf(hstr, "BIT8");
         break;
      case ECT_VISIBLE_STRING:
         sprintf(hstr, "VISIBLE_STRING");
         break;
      case ECT_OCTET_STRING:
         sprintf(hstr, "OCTET_STRING");
         break;
      default:
         sprintf(hstr, "Type 0x%4.4X", dtype);
   }
   return hstr;
}               

char* EcMaster::SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
   int l = sizeof(usdo) - 1, i;
   uint8 *u8;
   int8 *i8;
   uint16 *u16;
   int16 *i16;
   uint32 *u32;
   int32 *i32;
   uint64 *u64;
   int64 *i64;
   float *sr;
   double *dr;
   char es[32];
   
   memset(&usdo, 0, 128);
   ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
   if (EcatError)
   {
      return ec_elist2string();
   }
   else
   {
      switch(dtype)
      {
         case ECT_BOOLEAN:
            u8 = (uint8*) &usdo[0];
            if (*u8) sprintf(hstr, "TRUE"); 
                 else sprintf(hstr, "FALSE");
                 break;
         case ECT_INTEGER8:
            i8 = (int8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %d", *i8, *i8); 
            break;
         case ECT_INTEGER16:
            i16 = (int16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %d", *i16, *i16); 
            break;
         case ECT_INTEGER32:
         case ECT_INTEGER24:
            i32 = (int32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %d", *i32, *i32); 
            break;
         case ECT_INTEGER64:
            i64 = (int64*) &usdo[0];
            sprintf(hstr, "0x%16.16llx %lld", *i64, *i64); 
            break;
         case ECT_UNSIGNED8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %u", *u8, *u8); 
            break;
         case ECT_UNSIGNED16:
            u16 = (uint16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %u", *u16, *u16); 
            break;
         case ECT_UNSIGNED32:
         case ECT_UNSIGNED24:
            u32 = (uint32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %u", *u32, *u32); 
            break;
         case ECT_UNSIGNED64:
            u64 = (uint64*) &usdo[0];
            sprintf(hstr, "0x%16.16llx %llu", *u64, *u64); 
            break;
         case ECT_REAL32:
            sr = (float*) &usdo[0];
            sprintf(hstr, "%f", *sr); 
            break;
         case ECT_REAL64:
            dr = (double*) &usdo[0];
            sprintf(hstr, "%f", *dr); 
            break;
         case ECT_BIT1:
         case ECT_BIT2:
         case ECT_BIT3:
         case ECT_BIT4:
         case ECT_BIT5:
         case ECT_BIT6:
         case ECT_BIT7:
         case ECT_BIT8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%x", *u8); 
            break;
         case ECT_VISIBLE_STRING:
            strcpy(hstr, usdo);
            break;
         case ECT_OCTET_STRING:
            hstr[0] = 0x00;
            for (i = 0 ; i < l ; i++)
            { 
               sprintf(es, "0x%2.2x ", usdo[i]);
               strcat( hstr, es);
            }
            break;
         default:
            sprintf(hstr, "Unknown type");
      }
      return hstr;
   }
}

int EcMaster::si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;
    uint8 bitlen, obj_subidx;
    uint16 obj_idx;
    int abs_offset, abs_bit;

    rdl = sizeof(rdat); rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
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
		    rdl = sizeof(rdat2); rdat2 = 0;
		    /* read SDO that is mapped in PDO */
		    wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
		    rdat2 = etohl(rdat2);
		    /* extract bitlength of SDO */
		    bitlen = LO_BYTE(rdat2);
		    bsize += bitlen;
		    obj_idx = (uint16)(rdat2 >> 16);
		    obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
		    abs_offset = mapoffset + (bitoffset / 8);
		    abs_bit = bitoffset % 8;
		    ODlist.Slave = slave;
		    ODlist.Index[0] = obj_idx;
		    OElist.Entries = 0;
		    wkc = 0;
		    /* read object entry from dictionary if not a filler (0x0000:0x00) */
		    if(obj_idx || obj_subidx)
			wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
		    fprintf(pFile,"  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
		    if((wkc > 0) && OElist.Entries)
		    {
			fprintf(pFile," %-12s %s\n", dtype2string(OElist.DataType[obj_subidx]), OElist.Name[obj_subidx]);
		    }
		    else
			fprintf(pFile,"\n");
		    bitoffset += bitlen;
		};
	    };
	};
    };
    /* return total found bitlength (PDO) */
    return bsize;
}

int EcMaster::si_map_sdo(int slave)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize, outputs_bo, inputs_bo;
    uint8 SMt_bug_add;

    fprintf(pFile,"PDO mapping according to CoE :\n", 0x1B,1);
    SMt_bug_add = 0;
    outputs_bo = 0;
    inputs_bo = 0;
    rdl = sizeof(nSM); nSM = 0;
    /* read SyncManager Communication Type object count */
    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (nSM > 2))
    {
	/* make nSM equal to number of defined SM */
	nSM--;
	/* limit to maximum number of SM defined, if true the slave can't be configured */
	if (nSM > EC_MAXSM)
	    nSM = EC_MAXSM;
	/* iterate for every SM type defined */
	for (iSM = 2 ; iSM <= nSM ; iSM++)
	{
	    rdl = sizeof(tSM); tSM = 0;
	    /* read SyncManager Communication Type */
	    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
	    if (wkc > 0)
	    {
		if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
		{
		    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
		    fprintf(pFile,"Activated SM type workaround, possible incorrect mapping.\n");
		}
		if(tSM)
		    tSM += SMt_bug_add; // only add if SMt > 0
		
		if (tSM == 3) // outputs
		{
		    /* read the assign RXPDO */
		    fprintf(pFile,"  SM%1d outputs\n     addr b   index: sub bitl data_type    name\n", iSM);
		    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].outputs - (uint8 *)&m_IOmap[0]), outputs_bo );
		    outputs_bo += Tsize;
		}   
		if (tSM == 4) // inputs
		{
		    /* read the assign TXPDO */
		    fprintf(pFile,"  SM%1d inputs\n     addr b   index: sub bitl data_type    name\n", iSM);
		    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].inputs - (uint8 *)&m_IOmap[0]), inputs_bo );
		    inputs_bo += Tsize;
		}   
	    }   
	}
    }

    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
	retVal = 1;
    return retVal;
}

int EcMaster::si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset)
{
    uint16 a , w, c, e, er, Size;
    uint8 eectl;
    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint8 bitlen;
    int totalsize;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;
    char str_name[EC_MAXNAME + 1];

    eectl = ec_slave[slave].eep_pdi;
    Size = 0;
    totalsize = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
    if (t > 1)
	t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
	a = PDO->Startpos;
	w = ec_siigetbyte(slave, a++);
	w += (ec_siigetbyte(slave, a++) << 8);
	PDO->Length = w;
	c = 1;
	/* traverse through all PDOs */
	do
	{
	    PDO->nPDO++;
	    PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
	    PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
	    PDO->BitSize[PDO->nPDO] = 0;
	    c++;
	    /* number of entries in PDO */
	    e = ec_siigetbyte(slave, a++);
	    PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
	    a++;
	    obj_name = ec_siigetbyte(slave, a++);
	    a += 2;
	    c += 2;
	    if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
	    {   
		str_name[0] = 0;
		if(obj_name)
		    ec_siistring(str_name, slave, obj_name);                 
		if (t)
		    fprintf(pFile,"  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
		else
		    fprintf(pFile,"  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
		fprintf(pFile,"     addr b   index: sub bitl data_type    name\n");
		/* read all entries defined in PDO */
		for (er = 1; er <= e; er++)
		{
		    c += 4;
		    obj_idx = ec_siigetbyte(slave, a++);
		    obj_idx += (ec_siigetbyte(slave, a++) << 8);
		    obj_subidx = ec_siigetbyte(slave, a++);
		    obj_name = ec_siigetbyte(slave, a++);
		    obj_datatype = ec_siigetbyte(slave, a++);
		    bitlen = ec_siigetbyte(slave, a++);
		    abs_offset = mapoffset + (bitoffset / 8);
		    abs_bit = bitoffset % 8;

		    PDO->BitSize[PDO->nPDO] += bitlen;
		    a += 2;

		    str_name[0] = 0;
		    if(obj_name)
			ec_siistring(str_name, slave, obj_name);                 

		    fprintf(pFile,"  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
		    fprintf(pFile," %-12s %s\n", dtype2string(obj_datatype), str_name);
		    bitoffset += bitlen;
		    totalsize += bitlen;
		}
		PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
		Size += PDO->BitSize[PDO->nPDO];
		c++;
	    }
	    else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
	    {
		c += 4 * e;
		a += 8 * e;
		c++;
	    }   
	    if (PDO->nPDO >= (EC_MAXEEPDO - 1)) c = PDO->Length; /* limit number of PDO entries in buffer */
	}
	while (c < PDO->Length);
    }
    if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return totalsize;
}

int EcMaster::si_map_sii(int slave)
{
    int retVal = 0;
    int Tsize, outputs_bo, inputs_bo;

    fprintf(pFile,"PDO mapping according to SII :\n");

    outputs_bo = 0;
    inputs_bo = 0;
    /* read the assign RXPDOs */
    Tsize = si_siiPDO(slave, 1, (int)(ec_slave[slave].outputs - (uint8*)&m_IOmap), outputs_bo );
    outputs_bo += Tsize;
    /* read the assign TXPDOs */
    Tsize = si_siiPDO(slave, 0, (int)(ec_slave[slave].inputs - (uint8*)&m_IOmap), inputs_bo );
    inputs_bo += Tsize;
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
	retVal = 1;
    return retVal;
}


void EcMaster::si_sdo(int cnt)
{
    int i, j;
    
    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if( ec_readODlist(cnt, &ODlist))
    {
	fprintf(pFile," CoE Object Description found, %d entries.\n",ODlist.Entries);
	for( i = 0 ; i < ODlist.Entries ; i++)
	{
	    ec_readODdescription(i, &ODlist); 
	    while(EcatError) fprintf(pFile,"%s", ec_elist2string());
	    fprintf(pFile," Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
		ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i], ODlist.Name[i]);
	    memset(&OElist, 0, sizeof(OElist));
	    ec_readOE(i, &ODlist, &OElist);
	    while(EcatError) fprintf(pFile,"%s", ec_elist2string());
	    for( j = 0 ; j < ODlist.MaxSub[i]+1 ; j++)
	    {
		if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
		{
		    fprintf(pFile,"  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x Name: %s\n",
			j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j], OElist.Name[j]);
		    if ((OElist.ObjAccess[j] & 0x0007))
		    {
			fprintf(pFile,"          Value :%s\n", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
		    }
		}
	    }   
	}   
    }
    else
    {
	while(EcatError) fprintf(pFile,"%s", ec_elist2string());
    }
}

/*To write data of the found slaves in SlaveInfo.txt. Usefull to be sure of the slaves' order*/
void EcMaster::slaveInfo()
{
    pFile = fopen ("SlaveInfo.txt","w");
    ec_configdc();

    int cnt, i, j, nSM;
    uint16 ssigen, dtype;
    for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
			    
    {
	fprintf(pFile,"\nSlave:%d\n Name:%s\n PDO Output size: %dbits\n PDO Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
		    cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
		    ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
	if (ec_slave[cnt].hasdc) fprintf(pFile," DCParentport:%d", ec_slave[cnt].parentport);
	fprintf(pFile," Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
									(ec_slave[cnt].activeports & 0x02) > 0 , 
									(ec_slave[cnt].activeports & 0x04) > 0 , 
									(ec_slave[cnt].activeports & 0x08) > 0 );
	fprintf(pFile," Configured address: %4.4x\n", ec_slave[cnt].configadr);
	fprintf(pFile," Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
	for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
	{
		if(ec_slave[cnt].SM[nSM].StartAddr > 0)
			fprintf(pFile," SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n",nSM, ec_slave[cnt].SM[nSM].StartAddr, ec_slave[cnt].SM[nSM].SMlength,
				(int)ec_slave[cnt].SM[nSM].SMflags, ec_slave[cnt].SMtype[nSM]);
	}
	for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
	{
		fprintf(pFile," FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
			    (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
			    ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
			    ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
	}
	fprintf(pFile," FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
		ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
	fprintf(pFile," MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
	ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
	/* SII general section */
	if (ssigen)
		{
		    ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
		    ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
		    ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
		    ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
		    if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
		    {
			    ec_slave[cnt].blockLRW = 1;
			    ec_slave[0].blockLRW++;						
		    }	
		    ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
		    ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
		    ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
		}
	fprintf(pFile," CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
		    ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
	fprintf(pFile," Ebus current: %d[mA]\n only LRD/LWR:%d\n",
		    ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
	if ((ec_slave[cnt].mbx_proto & 0x04) && printSDO)
		si_sdo(cnt);
	    if(printMAP)
	{
		if (ec_slave[cnt].mbx_proto & 0x04)
		    si_map_sdo(cnt);
		else
		    si_map_sii(cnt);
	}  
    }
    
    fclose (pFile);
	    
    std::cout << "Slaves Info has been written to SlaveInfo.txt" << std::endl;
			
}
    
};///end of the namespace cpp4ec
