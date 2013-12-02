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



//ControlWord commands SGDV
enum
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
};

//StatusWord values SGDV
enum
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
};



namespace ec4cpp
{
class EcSlaveSGDV: public EcSlave
{
public:
    EcSlaveSGDV (ec_slavet* mem_loc);
    ~EcSlaveSGDV();

    bool configure() throw(EcErrorSGDV);
    void update();

    bool writeControlWord (uint16_t controlWord);
    bool readStatusWord (uint16_t statusWord);
    bool writeVelocity (int32_t velocity);
    bool readVelocity (int32_t velocity);

private:

    //Variables used for the properties
    bool useDC;
    unsigned int PDOerrorsTolerance;
    unsigned int SYNC0TIME;
    unsigned int SHIFT;
    unsigned int SHIFTMASTER;

    std::vector <parameter> m_params;

};
}
#endif