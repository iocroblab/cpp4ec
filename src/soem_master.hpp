#ifndef SOEM_MASTER_H
#define SOEM_MASTER_H

#include "soem_slave.hpp"
#include "soem_driver_factory.h"


#include <vector>
#include <string>
#include <sstream>
#include <sys/time.h>
#include <time.h>
#include <cstdio>
#include <unistd.h>
#include <stdint.h>


//ControlWord defines
#define CW_QUICK_STOP           0x02
#define CW_SHUTDOWN             0x06
#define CW_SWITCH_ON            0x07
#define CW_ENABLE_OP            0x0F
#define CW_DIASABLE_OP          0x07
#define CW_DISABLE_VOLTAGE      0x00
#define CW_FAULT_RESET          0x80
//Only for control position mode
#define CW_START_POSITIONING    0x1F
//StatusWord defines
#define SW_NOT_READY_SWICH_ON   0x00
#define SW_SWITCH_ON_DISABLED   0x40
#define SW_READY_SWITCH_ON      0x31
#define SW_SWITCHED_ON          0x33
#define SW_OPERATION_ENABLED    0x37
#define SW_QUICK_STOP_ACTIVE    0x07
#define SW_FAULT                0x08
#define SW_FAULT_RACTION_ACTIVE 0x0F
#define SW_HIGH_MASK            0x00FF
#define SW_LOW_MASK             0xFF00

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



namespace servos
{

class SoemMaster
{
    friend class Platform;

public:
    /**
     * \brief Constructor
     * 
     */ 
    SoemMaster();
    /**
     *  \brief Destructor
     */
    ~SoemMaster();
    
    /**
     *  \brief preConfiguration
     * 
     * Configures the master and slaves. In this functions is setted to Operational the EtherCAT State Machine.
     * 
     */
    bool preconfigure();
    
    /**
     *  \brief Configuration
     * 
     * Configures the master and slaves. In this functions is setted to Operational the EtherCAT State Machine.
     * 
     */
    bool configure();
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
    bool reset();
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
    

     /**
     *  \brief Sets master parameters
     * 
     * Configures the master parameters to work propertly
     * \param _cycletime set the period of the realtime task (PDO comunication)
     * \param _maxPvel is the maximum velocity that a motor can reach in milidegrees/second
     * \param _Pacc is the acceleration that motors apply to reach the desired velocity (useful for Operational Mode 3)
     * \param _Pdec is the deceleration that motors apply to reach the desired velocity (useful for Operational Mode 3)
     * 
     */
    inline void setParameters (int _cycletime, int8_t _opMode,
                               uint32_t _maxPvel,
                               uint32_t _Pacc,
                               uint32_t _Pdec) {
        cycletime = _cycletime;
        maxPvel = _maxPvel;
        Pacc = _Pacc;
        Pdec = _Pdec;
        opMode = _opMode;
    }

private:
     /**
     *  \brief Protected block
     * 
     * This block contains the private attributes and methods
     */
    std::string ethPort;
    char * ecPort;
    char m_IOmap[4096];
    int cycletime;	//the periodicity of ethercatLoop ("PDOs period")
    int8_t opMode;	//operationMode in which motors are controled
    uint32_t maxPvel;	// Maximum Profile Velocity
    uint32_t Pacc;  	// Profile acceleration
    uint32_t Pdec; 	// Profile Decceleration
    uint32_t Pqdec;	// Profile Quick Decceleration
    
    std::vector<SoemDriver*> m_drivers;


    //internal functions
    // This will be the main communication loop to be executed in a realtime separate thread
    //static void ethercatLoop(void *unused);
    bool switchState (int state); //switch the state of state machine--returns true if the state is reached
    int motor_control (uint16_t controlWord); //switch the motor state
    bool motion_setting (void); //configures operationMode and SDO paramenters
    int PDOmapping (void); //configures the PDO mapping



};

} //endnamespace Vector3d v;

#endif

