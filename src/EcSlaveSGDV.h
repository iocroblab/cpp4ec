#ifndef ECSLAVESGDV_H
#define ECSLAVESGDV_H

#include "EcSlave.h"
#include "EcError.h"
#include "EcErrorSGDV.h"
extern "C"
{
#include <soem/ethercattype.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/nicdrv.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>
}

//#include <boost/signals2/signal.hpp>
#include <vector>
#include <mutex> 


typedef struct
{
    std::string name;
    unsigned int index;
    unsigned int offset;
    unsigned int byteSize;
    unsigned int entry;
    std::string type;
}PDOobject;    

namespace cpp4ec
{
/**
 * \brief Class EcSlaveSGDV.
 * 
 * The class EcSlaveSGDV configures and controls the SGDV motors.  
 */
class EcSlaveSGDV: public EcSlave
{
public:
    //PDO Entries
    /**
     * The index of the PDO entries
     */
    typedef enum
    {
	FIRST_ENTRY, 	
	SECOND_ENTRY,	
	THIRD_ENTRY,	
	FOURTH_ENTRY,	
	FIFTH_ENTRY,
	SIXTH_ENTRY,	
	SEVENTH_ENTRY,	
    EIGHTH_ENTRY
    } EcPDOEntry;
    
     /**
     * The ControlWord values. The positioning words are only used in the profile position mode (operational mode = 1).
     */
    typedef enum
    {
	CW_QUICK_STOP         =  0x02,
	CW_SHUTDOWN           =  0x06, 
	CW_SWITCH_ON          =  0x07,
	CW_ENABLE_OP          =  0x0F, 
	CW_DIASABLE_OP        =  0x07,
	CW_DISABLE_VOLTAGE    =  0x00,
	CW_FAULT_RESET        =  0x80, 
	CW_HALT               =  0x09, 
	CW_START_ABS_POSITIONING  =  0x1F,	 /**< Start the next positioning after the current positioning completes
						     (target reached). The target position is an absolute value */
	CW_START_REL_POSITIONING  =  0x5F,	 /**< Start the next positioning after the current positioning completes
						     (target reached). The target position is a relative value */
	CW_START_QUICK_ABS_POSITIONING  =  0x3F, /**< Start next positioning immediately. The target position is an absolute value */
	CW_START_QUICK_REL_POSITIONING  =  0x7F, /**< Start next positioning immediately. The target position is a relative value */
	CW_START_ABS_POSITIONING_WITH_VELOCITY  =  0x21F, /**< Positioning with current profile velocity up to the 
								current set-point is proceeded and then next positioning will be applied.
								The target position is an absolute value */
	CW_START_REL_POSITIONING_WITH_VELOCITY  =  0x25F, /**< Positioning with current profile velocity up to the 
								current set-point is proceeded and then next positioning will be applied.
								The target position is a relative value */
    } EcControlWord;

     /**
     * The StatusWord  values
     */
    typedef enum
    {
	SW_NOT_READY_SWICH_ON   =   0x00,
	SW_SWITCH_ON_DISABLED   =   0x40,
	SW_READY_SWITCH_ON      =   0x31,
	SW_SWITCHED_ON          =   0x33,
	SW_OPERATION_ENABLED    =   0x37,
	SW_QUICK_STOP_ACTIVE    =   0x07,
	SW_FAULT                =   0x08,
	SW_FAULT_RACTION_ACTIVE =   0x0F,
	SW_HIGH_MASK            = 0x00FF,
	SW_LOW_MASK             = 0xFF00,
    } EcStatusWord;
    
    /**
     * \brief Constructor
     * 
     * \param mem_loc A posinter to the slave information of ec_slave (SOEM)
     */ 
    EcSlaveSGDV (ec_slavet* mem_loc);
    
    /**
     * \brief Destructor
     * 
     */
    ~EcSlaveSGDV();
    
    /**
     * \brief Gets the name
     * 
     */
    const std::string& getName() const;
    
    /**
     * \brief Configure the SGDV servopack
     * 
     * Set the parameters readed from the XML, which include the PDO mapping.
     */
    bool configure() throw(EcErrorSGDV);
    
    /**
     * \brief Start motors
     * 
     * Switch on the motors.
     * \return A vector with the secuence of buffer outputs that has to be sent to stop the servos
     * 
     */
     void start() throw(EcErrorSGDV);
    
    /**
     * \brief Update the data
     * 
     * Update the output and input values to and from their respective buffers.
     * 
     */
    void update(); 
    
    /**
     * \brief Stop motors
     * 
     * \return A vector with the secuence of buffer outputs that has to be sent to stop the servos
     * 
     */
     void stop() throw(EcErrorSGDV);
     
     /**
     * \brief Set DC
     * 
     * Configure the sync0 event
     * 
     * \param active true to active DC, false to disconect.
     * \param sync0Time the cycle time of the sync0 event
     * \param sync0Shift the shifted time of the sync0 event  
     * 
     */
     void setDC(bool active, unsigned int sync0Time, unsigned int sync0Shift) throw(EcErrorSGDV);

    /**
     * \brief Sets locations Buffer
     * 
     * The master join all the outputs and inputs in their respective buffer to transmit to the real time task.
     * This functions sets the input and output pointers to the right location on the where the PDO must be put. 
     * 
     * \param input the pointer to the right location on the master input buffer
     * \param output the pointer to the right location on the master input buffer
     */
    void setPDOBuffer(char * input, char * output);
    
    /**
    * \brief Writes on the output PDOentry
    * 
    *  
    * \param entry especifies the entry that occupies the object thatis wanted to write on.
    * \param value the desired value of the onject.
    * 
    */
    bool writePDO (EcPDOEntry entry, int value);
    
    /**
    * \brief Read on the input PDOentry
    * 
    * \param entry especifies the entry that occupies the object that is wanted to read on.
    * \param value return the value of this onject.
    * 
    */
    bool readPDO (EcPDOEntry entry, int &value);
    
    /**
    * \brief Write on the Controlword object
    * 
    * It's an specific function to write on the Controlword object (0x6040). If this object is not mapped in the PDO, then an error will happen.
    * \param controlWord the desired value of the Controlword object .
    * 
    */
    bool writeControlWord (uint16_t controlWord);
    
    /**
    * \brief Read the Statusword object
    * 
    * It's an specific function to read the Statusword object (0x6041). If this object is not mapped in the PDO, then an error will happen.
    * \param statusWord return the value of the Statusword object .
    * 
    */
    bool readStatusWord (uint16_t &statusWord);

    /**
    * \brief Write on the position object
    * 
    * It's an specific function to write on the Target Position object (0x607A). If this object is not mapped in the PDO, then an error will happen.
    * \param position the desired value of the Target Position object.
    * 
    */
    bool writePosition (int32_t position);
    
    /**
    * \brief Read the position object
    * 
    * It's an specific function to read the Position Actual Value object (0x6064). If this object is not mapped in the PDO, then an error will happen.
    * \param position return the value of the Position Actual Value object .
    * 
    */
    bool readPosition (int32_t &position);
    
    /**
    * \brief Write on the velocity object
    * 
    * It's an specific function to write on the Target Velocity object (0x60FF). If this object is not mapped in the PDO, then an error will happen.
    * \param velocity the desired value of the Target Velocity object.
    * 
    */
    bool writeVelocity (int32_t velocity);
    
    /**
    * \brief Read the velocity object
    * 
    * It's an specific function to read the Velocity Actual Value object (0x606C). If this object is not mapped in the PDO, then an error will happen.
    * \param velocity return the value of the Velocity Actual Value object .
    * 
    */
    bool readVelocity (int32_t &velocity);
    
    /**
    * \brief Write on the torque object
    * 
    * It's an specific function to write on the Target Torque object (0x6071). If this object is not mapped in the PDO, then an error will happen.
    * \param torque the desired value of the Target Torque object.
    * 
    */
    bool writeTorque (int16_t torque);
    
    /**
    * \brief Read the torque object
    * 
    * It's an specific function to read the Torque Actual Value object (0x6077). If this object is not mapped in the PDO, then an error will happen.
    * \param torque return the value of the Torque Actual Value object .
    * 
    */
    bool readTorque (int16_t &torque);
    
    /**
    * \brief Read the actual timestamp
    * 
    * Reads the timestamp of the last PDO readed.
    * \param time return the value of the actual timestamp.
    * 
    */
    bool readTimestamp (unsigned long& time);
    /**
    * \brief Set an SGDV object
    * 
    * This functions set the desired value on the desired object of an SGDV Servopack.
    * \param index the index of the object.
    * \param subindex the subindedex of the objec.
    * \param psize the size of the param buffer.
    * \param param a buffer with the desired value.
    * 
    */
    void setSGDVObject(uint16_t index, uint8_t subindex, int psize, void * param);
    
    /**
    * \brief Set an SGDV object
    * 
    * This functions set the desired value on the desired object of an SGDV Servopack.
    * \param index the index of the object.
    * \param subindex the subindedex of the objec.
    * \param psize the size of the param buffer.
    * \param param a pointer to the buffer with the returned value.
    */
    void getSGDVObject(uint16_t index, uint8_t subindex, int *psize, void *param); 
    
    
     /**
    * \brief A signal to emit information
    * 
    * Is a function designed to send the important data of the slave. Whatever component that is conected to this signal
    * can recieve the slavenumber, statusWord, Position, Velocity, Torque and timestamp of each slave.
    * 
    */
    boost::signals2::signal<void (int, uint16_t, int32_t, int32_t, int16_t, unsigned long)> slaveValues;
    
   



private:
    
    int outputSize;
    int inputSize;
#ifdef RTNET
    char* pBufferOut;
    char* pBufferIn;
#else
    uint8* pBufferOut;
    uint8* pBufferIn;
#endif
    char* inputBuf;
    
    bool readXML() throw(EcErrorSGDV);
    bool enableSpecificFunctions();
    void loadDefaultPDO();
    void loadParameters();
    void si_PDOassign(uint16 slave, uint16 PDOassign);

    
    int controlWordEntry;
    int targetPositionEntry;
    int targetVelocityEntry;
    int targetTorqueEntry;
    int statusWordEntry;
    int actualPositionEntry;
    int actualVelocityEntry;
    int actualTorqueEntry;
    
    bool wControlWordCapable;
    bool wPositionCapable;
    bool wVelocityCapable;
    bool wTorqueCapable;
    bool rStatusWordCapable;
    bool rPositionCapable;
    bool rVelocityCapable;
    bool rTorqueCapable;
    bool parameterSetting;
    bool PDOmapping;
    
    std::mutex slaveInMutex;
    std::mutex slaveOutMutex;

    std::vector <CoEparameter> m_params;
    std::vector <PDOobject> inputObjects;
    std::vector <PDOobject> outputObjects;
    
    //PDO objects
    typedef enum
    {
	CONTROL_WORD 		= 0x6040,
	TARGET_POSITION 	= 0x607A,
	TARGET_VELOCITY 	= 0x60FF,
	TARGET_TORQUE 		= 0x6071,
    }EcRecieveObjects;


    typedef enum
    {
	STATUS_WORD 		= 0x6041,
	ACTUAL_POSITION 	= 0x6064,
	ACTUAL_VELOCITY 	= 0x606C,
	ACTUAL_TORQUE 		= 0x6077,
    }EcTransmitObjects;
};
}
#endif
