#ifndef ECMASTER_H
#define ECMASTER_H

#include "EcSlave.h"
#include "EcSlaveFactory.h"
#include "EcError.h"


extern "C"
{
#include <soem/ethercattype.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>
#include <soem/nicdrv.h>
}

#include <vector>
#include <string>
#include <sstream>
#include <sys/time.h>
#include <time.h>
#include <cstdio>
#include <unistd.h>
#include <stdint.h>
#include <mutex> 
#include <thread>

#define NSEC_PER_SEC 1000000000



namespace cpp4ec
{
/**
 * 
 * \brief Class EcMaster.
 * 
 * The EcMaster is the manager of the EtherCat state machine and communications.These are examples of how to use the EcMaster class:
 * 
 */
class EcMaster
{

public:
    /**
     * \brief Constructor 
     *
     * \param cycleTime The period of the RT task that send PDO.
     * \param useDC if true the slaves use DC synchronization.
     * \param slaveInfo if true prints on EtherCATsoemInfo.txt the slave information.
     *    
     */
    EcMaster(unsigned long cycleTime = 1000000, bool useDC = false, bool slaveInfo = false);

    /**
     *  \brief Destructor
     */
    ~EcMaster();


    /**
     *  \brief Configuration
     *
     * Configures the master and slaves. In this functions is setted to Operational the EtherCAT State Machine.
     *
     */
    bool configure() throw(EcError);

    /**
     *  \brief Starts comunication
     *
     * The realtime task for comunication starts sendding PDOs and the mottor are switched on.
     */
    bool start() throw(EcError);

    /**
     *  \brief Stops communication
     *
     * The realtime task for comunication is stopped and the mottors are shutted down.
     */
    bool stop() throw(EcError);

    /**
     *  \brief Resets configuration
     *
     * The EtherCAT State Machine is taken to the initial state.
     */
    bool reset() throw(EcError);

    
    /**
     *  \brief Get slaves vector
     *
     *	\return the slaves vector.
     */
    std::vector<EcSlave*> getSlaves();
    
    /**
     *  \brief Update the outputs
     *
     *	Update the output values. It sends the output PDO to the RT task, to be sent to devices.
     */
    void update(void) throw(EcError);
    
    /**
     * 
     * \example demo1.cpp
     *  
     * \example demo2.cpp
     * 
     * \example demo3.cpp
     *
     * \example demo4.cpp
     * 
     */
   


private:
     /**
     *  \brief Protected block
     *
     * This block contains the private attributes and methods
     */
    
     //Master variables
    std::string ethPort;
    char * ecPort;
    char m_IOmap[4096];
    unsigned long m_cycleTime;	//the periodicity of ethercatLoop ("PDOs period")
    bool m_useDC;
    std::vector<EcSlave*> m_drivers;
    char * inputBuf;
    char * outputBuf;
    int inputSize, outputSize;
    
    
    //ethercat switchState function
    bool switchState (ec_state state); //switch the state of state machine--returns true if the state is reached
    
    //Sockect stuff    
    bool threadFinished;
    char * devnameOutput;
    char * devnameInput;
    int fdOutput,fdInput;
    std::thread updateThread;
    void update_EcSlaves(void) throw(EcError);
    
    int unusedVariable;

    //Ethercat slaveinfo stuff
    bool slaveInformation;
    FILE * pFile;
    ec_ODlistt ODlist;
    ec_OElistt OElist;
    char usdo[128];
    char hstr[1024];
    bool printSDO;
    bool printMAP;
    void slaveInfo(void);
    char* dtype2string(uint16 dtype);
    char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype);
    int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset);
    int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);
    int si_map_sdo(int slave);
    int si_map_sii(int slave);
    void si_sdo(int cnt);
};
}
#endif

