#ifndef ECMASTER_H
#define ECMASTER_H

#include "EcSlave.h"
#include "EcSlaveFactory.h"

#include <vector>
#include <string>
#include <sstream>
#include <sys/time.h>
#include <time.h>
#include <cstdio>
#include <unistd.h>
#include <stdint.h>

#include "EcError.h"

#define NSEC_PER_SEC 1000000000


/*Structure to know the activated DC slaves*/
struct slaveDCspec {
    unsigned int slaveNumber;
    bool state;
    unsigned int cycleTime;
    unsigned int shift;
};


// template<class T>
// inline std::string to_string (const T& t, std::ios_base & (*f) (std::ios_base&))
// {
//     std::stringstream ss;
//     ss << f << t;
//     return ss.str();
// };

//slave info variables
// ec_ODlistt ODlist;
// ec_OElistt OElist;
// boolean printSDO = FALSE;
// boolean printMAP = FALSE;
// char usdo[128];
// char hstr[1024];

void slaveinfo();

namespace ec4cpp
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
    bool start();

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

    /**
     *  \brief Sets Position
     */
    bool setPosition (std::vector <int32_t>&pos);

    /**
     *  \brief Gets Position
     */
    bool getPosition (std::vector <int32_t>&pos);

    /**
     *  \brief Sets Velocity
     *
     * Writes on the PDO the desired velocity for each motor in milidegrees/second.
     */
    bool setVelocity (std::vector <int32_t>&vel);

    /**
     *  \brief Gets Velocity
     *
     * Reads on the PDO the velocity of each motor in milidegrees/second.
     */
    bool getVelocity (std::vector <int32_t>&vel);




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

    bool switchState (ec_state state); //switch the state of state machine--returns true if the state is reached

};

} //endnamespace Vector3d v;

#endif

