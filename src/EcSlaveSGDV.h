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

#include <vector>
#include <iostream>

//PDO Entries
typedef enum
{
    firstEntry, // first = 0
    secondEntry,
    thirdEntry,
    fouthEntry,
    fithEntry,
    sixthEntry,
    seventhEntry,
    eighthEntry,
}EcPDOEntry;

//ControlWord commands SGDV
typedef enum
{
   CW_QUICK_STOP         =  0x02,
   CW_SHUTDOWN           =  0x06,
   CW_SWITCH_ON          =  0x07,
   CW_ENABLE_OP          =  0x0F,
   CW_DIASABLE_OP        =  0x07,
   CW_DISABLE_VOLTAGE    =  0x00,
   CW_FAULT_RESET        =  0x80,
   //Only for control position mode
   CW_START_POSITIONING  =  0x1F,
}EcControlWord;

//StatusWord values SGDV
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
class EcSlaveSGDV: public EcSlave
{
public:
    EcSlaveSGDV (ec_slavet* mem_loc);
    ~EcSlaveSGDV();

    bool configure() throw(EcErrorSGDV);
    void start() throw(EcErrorSGDV);
    void stop() throw(EcErrorSGDV);

    bool writePDO (EcPDOEntry entry, int value);
    bool readPDO (EcPDOEntry entry, int &value);
    void setSGDVOject(uint16_t index, uint8_t subindex, int psize, void * param);
    void getSGDVObject(uint16_t index, uint8_t subindex, int *psize, void *param);

private:

    //Variables used for the properties
    bool useDC;
    unsigned int PDOerrorsTolerance;
    unsigned int SYNC0TIME;
    unsigned int SHIFT;
    unsigned int SHIFTMASTER;
        
    int transmitEntry;
    int recieveEntry;
    
    void readXML() throw(EcErrorSGDV);
    bool addPDOobject(std::string PDOentry,int value, int subindex);
    

    std::vector <parameter> m_params;
    std::vector <PDOobject> inputObjects;
    std::vector <PDOobject> outputObjects;

};
}
#endif