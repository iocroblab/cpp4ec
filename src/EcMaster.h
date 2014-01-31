#ifndef ECMASTER_H
#define ECMASTER_H

#include "EcSlave.h"
#include "EcSlaveFactory.h"

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

#include "EcError.h"

#define NSEC_PER_SEC 1000000000
// #define XDDP_PORT_INPUT "EcMaster-xddp-input"
// #define XDDP_PORT_OUTPUT "EcMaster-xddp-output"

/*Structure to know the activated DC slaves*/
struct slaveDCspec {
    unsigned int slaveNumber;
    bool state;
    unsigned int cycleTime;
    unsigned int shift;
};

//  ///realtime stuff
//     char * inputBuf;
//     char * outputBuf;
//     int inputSize, outputSize;
// 
    pthread_t nrt;

// template<class T>
// inline std::string to_string (const T& t, std::ios_base & (*f) (std::ios_base&))
// {
//     std::stringstream ss;
//     ss << f << t;
//     return ss.str();
// };




namespace cpp4ec
{

    
class EcMaster
{


public:
    /**
     * \brief Constructor
     *
     */
    EcMaster(int cycleTime = 1000000);

    /**
     *  \brief Destructor
     */
    ~EcMaster();

    /**
     *  \brief preConfiguration
     *
     * Configures the master and slaves. In this functions is setted to Operational the EtherCAT State Machine.
     *
     */
    bool preconfigure() throw(EcError);

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
    bool stop();

    /**
     *  \brief Resets configuration
     *
     * The EtherCAT State Machine is taken to the initial state.
     */
    bool reset() throw(EcError);

    
    
    std::vector<EcSlave*> getSlaves();




private:
     /**
     *  \brief Protected block
     *
     * This block contains the private attributes and methods
     */
    std::string ethPort;
    char * ecPort;
    char m_IOmap[4096];
    int m_cycleTime;	//the periodicity of ethercatLoop ("PDOs period")
    std::vector<EcSlave*> m_drivers;
    ec_ODlistt ODlist;
    ec_OElistt OElist;
    FILE * pFile;
    char * devnameOutput;
    int fdOutput;
    
    ///Ethercat stuff
    void slaveInfo();
    bool switchState (ec_state state); //switch the state of state machine--returns true if the state is reached
    int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset);
    int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);
    int si_map_sdo(int slave);
    int si_map_sii(int slave);
    void si_sdo(int cnt);
    
    //signal stuff
    sigset_t mask, oldmask;

   //realtime stuff 

   
    
//     pthread_t nrt;
//     static void cleanup_upon_sig(int sig)
//     static void update_EcSlaves(void *unused);
    void update_ec(void) throw(EcError);
    ///realtime stuff
    char * inputBuf;
    char * outputBuf;
    int inputSize, outputSize;
// 
//     static pthread_t nrt;
    void update_EcSlaves(void) throw(EcError);
    static void cleanup_upon_sig(int sig);
    
};

} //endnamespace Vector3d v;

#endif

