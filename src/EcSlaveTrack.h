#ifndef ECSLAVETRACK_H
#define ECSLAVETRACK_H

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
#include <soem/ethercatsoe.h>
#include <soem/ethercatprint.h>
}

#include <boost/signals2/signal.hpp>
#include <vector>
#include <iostream>
#include <mutex> 

//MTD objects
typedef enum
{
    CONTROL_WORD 	= 0x0086, //(S-0-0134)
    TARGET_POSITION 	= 0x002F, //(S-0-0047)
    TARGET_VELOCITY 	= 0x0024, //S-0-0036) 
    TARGET_TORQUE 	= 0x0050, //(S-0-0080)
}EcRecieveObjects;

//AT objects
typedef enum
{
    STATUS_WORD 	= 0x0087, //(S-0-0135)
    ACTUAL_POSITION_1 	= 0x0033, //(S-0-0051)
	ACTUAL_POSITION_2 	= 0x0035, //(S-0-0053)
    ACTUAL_VELOCITY 	= 0x0028, //(S-0-0040)
    ACTUAL_TORQUE 	= 0x0054, //(S-0-0084)
}EcTransmitObjects;

//MTD Entries
typedef enum
{
    FIRST_ENTRY, // first = 0
    SECOND_ENTRY,
    THIRD_ENTRY,
    FOURTH_ENTRY,
    FIFTH_ENTRY,
    SIXTH_ENTRY,
    SEVENTH_ENTRY,
    EIGHTH_ENTRY,
    NINTH_ENTRY,
    TENTH_ENTRY,
    ELEVENTH_ENTRY,
    TWELFTH_ENTRY,
    THIRTEENTH_ENTRY,
    FOURTEENTH_ENTRY,
    FIFTEENTH_ENTRY,
}EcPDOEntry;

typedef enum
{
    VZ0 =0, //MDT: no cyclic data, AT: no cyclic data
    VZ1 =1, //MDT: S-0-0080 torque command, AT: no cyclic data
    VZ2 =2, //MDT: S-0-0036 velocity command, AT: S-0-0040 velocity feedback
    VZ3 =3, //MDT: S-0-0036 velocity command, AT: S-0-0051/53 position feedback 1/2
    VZ4 =4, //MDT: S-0-0047 position command, AT: S-0-0051/53 position feedback 1/2
    VZ5 =5, //MDT: S-0-0047 position command and S-0-0036 velocity command, AT:S-0-0051/53 position feedback 1/2 and S-0-0040 velocity feedback
    VZ6 =6, //MDT: S-0-0036 velocity command, AT: no cyclic data
    VZ7 =7, //Configurable, you will need to set up the telegrams by yourself using the functions setMDT and setAT
}EcTelegramType;


typedef enum
{
    TORQUE_CONTROL =1,
    VELOCITY_CONTROL =2,
    POSITION_CONTROL =3,
    DRIVEINTERNALINTERPOLATION_CONTROL =19,
}EcMode;

//ControlWord S-0-0134
typedef enum
{
	
	CW_DRIVE_ENABLE        =  0x4000, //Bit 14 a 1, 
	CW_DRIVE_DISABLE       =  0x0000, //TODO A 0
	CW_DRIVE_ON            =  0xC000, //0x4000+0x8000 Bit 15 y 14 a 1,
	CW_DRIVE_OFF           =  0x4000, //bit 15 a 0, 14 a 1.
	CW_DRIVE_HALT          =  0xE000, //0x4000+0x8000+0x2000, bits 13,14,15 a 1
	CW_PRIMARY_MODE        =  0xC000, //
	CW_SECONDARY_MODE_1    =  0xC800,
	CW_SECONDARY_MODE_2    =  0xC200,
	CW_SECONDARY_MODE_3    =  0xCA00,
	CW_SECONDARY_MODE_4    =  0xC100,
	CW_SECONDARY_MODE_5    =  0xC900,
	CW_SECONDARY_MODE_6    =  0xC300,
	CW_SECONDARY_MODE_7    =  0xCB00,
	
   
}EcControlWord;

//StatusWord values CARRIL P-0-0116
/*typedef enum
{
	SW_BBRELAY_CONTROL					=   0x01,
	SW_DRIVE_READY						=   0x02,
	SW_DRIVE_FOLLOW_COMAND				=   0x08,
	SW_DRIVE_HALT_ON					=   0x10,
	SW_CHANGE_COMMAND_STATUS			=   0x20,
	SW_OPERATION_MODE_INITIALIZED		=   0x80,
	SW_PRIMARY_MODE						=   0x00, //BITS 8/9/11 a 0
	SW_SECONDARY_MODE_1					=   0x0800,
	SW_SECONDARY_MODE_2					=   0x0200,
	SW_SECONDARY_MODE_3					=   0x0A00,
	SW_SECONDARY_MODE_4					=   0x0100,
	SW_SECONDARY_MODE_5					=   0x0900,
	SW_SECONDARY_MODE_6					=   0x0300,
	SW_SECONDARY_MODE_7					=   0x0B00,
	SW_DRIVE_ERROR						=   0x2000,
	SW_CONTROL_READY			        =   0x8000,
	SW_CONTROL_POWER_READY				=	0x4000,
	SW_DRIVE_TORQUE						=	0xC000,
}EcStatusWord2;
*/

//StatusWord values CARRIL S-0-0135, hay que comprobar bit a bit, con un & o algo asi
typedef enum
{
	SW_NOT_READY						=   0x00, //drive not ready for power on, because internal checks not positively completed
	SW_READY							=   0x4000, //ready for power on
	SW_READY_FREE						=   0x8000, //control section and power section ready for operation, torque-free
	SW_READY_COMPL						=   0xC000, //in operation, with torque
	SW_CHANGE_COMMAND_STATUS			=   0x20,
	SW_OPERATION_MODE_INITIALIZED		=   0x80,
	SW_PRIMARY_MODE						=   0x00, //BITS 8/9/11 a 0
	SW_SECONDARY_MODE_1					=   0x0100,
	SW_SECONDARY_MODE_2					=   0x0200,
	SW_SECONDARY_MODE_3					=   0x0300,
	SW_SECONDARY_MODE_4					=   0x0400,
	SW_SECONDARY_MODE_5					=   0x0500,
	SW_SECONDARY_MODE_6					=   0x0600,
	SW_SECONDARY_MODE_7					=   0x0700,
	
}EcStatusWord;

typedef struct
{
    std::string name;
    unsigned int offset;
    unsigned int byteSize;
    std::string type;
}PDOobject;    

namespace cpp4ec
{
/**
 * \brief EcSlaveTrack.
 * 
 * The class EcSlaveTrack configures and controls the SGDV motors.
 */
class EcSlaveTrack: public EcSlave
{
public:
    /**
     * \brief Constructor
     * 
     * \param mem_loc A posinter to the slave information of ec_slave (SOEM)
     */ 
    EcSlaveTrack (ec_slavet* mem_loc);
    
    /**
     * \brief Destructor
     * 
     */
    ~EcSlaveTrack();
    
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
     * \brief Sets locations Buffer
     * 
     * The master join all the outputs and inputs in their respective buffer to transmit to the real time task.
     * This functions sets the input and output pointers to the right location on the where the PDO must be put. 
     * 
     * \param input the pointer to the right location on the master input buffer
     * \param output the pointer to the right location on the master input buffer
     */
    void setPDOBuffer(char * input, char * output);
    
    void setDC(bool active, unsigned int sync0Time, unsigned int sync0Shift) throw(EcErrorSGDV);

    /**
    * \brief Writes on the output PDOentry
    * 
    *  
    * \param entry especifies the entry that occupies the object thatis wanted to write on.
    * \param value the desired value of the onject.
    * 
    */
    bool writeMDT (EcPDOEntry entry, int value)throw(EcErrorSGDV);
    
    /**
    * \brief Read on the input PDOentry
    * 
    * \param entry especifies the entry that occupies the object that is wanted to read on.
    * \param value return the value of this onject.
    * 
    */
    bool readAT (EcPDOEntry entry, int &value);
    
    /**
    * \brief Write on the Controlword object
    * 
    * It's an specific function to write on the Controlword object (0x6040).If this object is not mapped in the PDO, then an error will happen.
    * \param controlWord the desired value of the Controlword object .
    * 
    */
    bool writeControlWord (uint16_t controlWord);
    
    /**
    * \brief Read the Statusword object
    * 
    * It's an specific function to read the Statusword object (0x6041).If this object is not mapped in the PDO, then an error will happen.
    * \param statusWord return the value of the Statusword object .
    * 
    */
    bool readStatusWord (uint16_t &statusWord);

    /**
    * \brief Write on the position object
    * 
    * It's an specific function to write on the Target Position object (0x607A).If this object is not mapped in the PDO, then an error will happen.
    * \param postion the desired value of the Target Position object.
    * 
    */
    bool writePosition (int32_t position);
    
    /**
    * \brief Read the position object
    * 
    * It's an specific function to read the Position Actual Value object (0x6064).If this object is not mapped in the PDO, then an error will happen.
    * \param position return the value of the Position Actual Value object .
    * 
    */
    bool readPosition (int32_t &position);
    
    /**
    * \brief Write on the velocity object
    * 
    * It's an specific function to write on the Target Velocity object (0x60FF).If this object is not mapped in the PDO, then an error will happen.
    * \param velocity the desired value of the Target Velocity object.
    * 
    */
    bool writeVelocity (int32_t velocity);
    
    /**
    * \brief Read the velocity object
    * 
    * It's an specific function to read the Velocity Actual Value object (0x606C).If this object is not mapped in the PDO, then an error will happen.
    * \param velocity return the value of the Velocity Actual Value object .
    * 
    */
    bool readVelocity (int32_t &velocity);
    
    /**
    * \brief Write on the torque object
    * 
    * It's an specific function to write on the Target Torque object (0x6071).If this object is not mapped in the PDO, then an error will happen.
    * \param torque the desired value of the Target Torque object.
    * 
    */
    bool writeTorque (int16_t torque);
    
    /**
    * \brief Read the torque object
    * 
    * It's an specific function to read the Torque Actual Value object (0x6077).If this object is not mapped in the PDO, then an error will happen.
    * \param torque return the value of the Torque Actual Value object .
    * 
    */
    bool readTorque (int16_t &torque);
    
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
    void setSGDVOject(uint16_t idn, uint8_t elementflags, int psize, void * param);
    
    /**
    * \brief Set an SGDV object
    * 
    * This functions set the desired value on the desired object of an SGDV Servopack.
    * \param index the index of the object.
    * \param subindex the subindedex of the objec.
    * \param psize the size of the param buffer.
    * \param param a pointer to the buffer with the returned value.
    */
    void getSGDVObject(uint16_t idn, uint8_t elementflags, int *psize, void *param); 

    /**
    * \brief Read the IDN that are contained in a IDN
    *
    * This functions reads the idn that are contained in a IDN that have IDN as values.
    * \param idn the idn that are going to be read.
    * \param print has to be set as true for printing the values and false for not printing the values.
    */
    std::vector<int> readIDNofIDNs(int idn, bool print);

    /**
    * \brief Set the IDN that will be contained in the MDT
    *
    * This functions set the IDN that will be contained in te MDT during the cyclic data transfer
    * \param listidn vector with all the parameters that wil be set in the MDT (the controlword has not to be set in this list).
    */
    void setMDT(std::vector<int> listidn);

    /**
    * \brief Set the IDN that will be contained in the AT
    *
    * This functions set the IDN that will be contained in te AT during the cyclic data transfer
    * \param listidn vector with all the parameters that wil be set in the AT (the statuslword has not to be set in this list).
    */
    void setAT(std::vector<int> listidn);

    /**
    * \brief Set the operation mode
    *
    * This functions set operation mode of the slave.
    * \param mode is the mode that will be set in the nmode position.
    * \param nmode position of the mode, you could configure up to 4 mode at the same time and change it while the slave is
    * in the cyclic data transfer mode using the controlword to switch between the mode on te fly. Set nmode with [0,1,2,3]
    * to configure the 4 modes simultaniosly.
    */
    void modeSetUp(EcMode mode, int nmode);

    /**
    * \brief Set the telegram type
    *
    * This functions set the telegram type, it is possible to choose between 6 congigurated telegrams and a configurable one.
    * \param type the type of telegram that are going to be set
    *       VZ0 MDT: no cyclic data, AT: no cyclic data
    *       VZ1 MDT: S-0-0080 torque command, AT: no cyclic data
    *       VZ2 MDT: S-0-0036 velocity command, AT: S-0-0040 velocity feedback
    *       VZ3 MDT: S-0-0036 velocity command, AT: S-0-0051/53 position feedback 1/2
    *       VZ4 MDT: S-0-0047 position command, AT: S-0-0051/53 position feedback 1/2
    *       VZ5 MDT: S-0-0047 position command and S-0-0036 velocity command, AT:S-0-0051/53 position feedback 1/2 and S-0-0040 velocity feedback
    *       VZ6 MDT: S-0-0036 velocity command, AT: no cyclic data
    *       VZ7 Configurable, you will need to set up the telegrams by yourself using the functions setMDT and setAT
    */
    void TelegramType(EcTelegramType type);

    void DefaultParameters();

    void ClearError();



    
//    void refresh();
    boost::signals2::signal<void (int, uint16_t, int32_t, int32_t, int16_t)> slaveValues;


private:

    //Variables used for the properties
    bool useDC;
    unsigned int PDOerrorsTolerance;
    unsigned int SYNC0TIME;
    unsigned int SHIFT;
    unsigned int SHIFTMASTER;
    
    int minPosition;
    int maxPosition;
    int m_mutex;
    
    int outputSize;
    int inputSize;
    
    int transmitEntry;
    int recieveEntry;
    
    void readXML() throw(EcErrorSGDV);
    bool addPDOobject(std::string PDOentry,int value, int subindex);
    
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
    
    char* outputBuf;
    char* inputBuf;
    char* inputNull;
    char* pBufferOut;
    char* pBufferIn;
    

    
    std::mutex slaveInMutex;
    std::mutex slaveOutMutex;

    std::vector <SoEparameter> m_params;
    std::vector <PDOobject> inputObjects;
    std::vector <PDOobject> outputObjects;
    std::vector <char*> bufferList;
};
}
#endif
