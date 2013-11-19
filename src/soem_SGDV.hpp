#ifndef OROCOS_SOEM_SGDV_HPP
#define OROCOS_SOEM_SGDV_HPP


#include <vector>
#include <iostream>

#include "soem_slave.hpp"
//Xenomai
#include <native/task.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>

#include "servos_rt.h"
extern "C"
{
#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>
}

//ControlWord defines
#define CW_QUICK_STOP           0x02
#define CW_SHUTDOWN             0x06
#define CW_SWITCH_ON            0x07
#define CW_ENABLE_OP            0x0F
#define CW_DIASABLE_OP          0x07
#define CW_DISABLE_VOLTAGE      0x00
#define CW_FAULT_RESET          0x80
//Only for the position mode
#define CW_START_POSITIONING    0x1F
//StatusWord defines
#define SW_NOT_READY_SWICH_ON   0x00
#define SW_SWITCH_ON_DISABLED   0x40
#define SW_READY_SWITCH_ON      0x21
#define SW_SWITCHED_ON          0x233
#define SW_OPERATION_ENABLED    0x237
#define SW_QUICK_STOP_ACTIVE    0x07
#define SW_FAULT                0x08
#define SW_FAULT_RACTION_ACTIVE 0x0F
//Mask for StatusWord comparison
#define SW_NOT_READY_SWICH_ON_MASK 0x4F
#define SW_SWITCH_ON_DISABLED_MASK 0x4F
#define SW_READY_SWITCH_ON_MASK    0x6F
#define SW_SWITCHED_ON_MASK        0x27F
#define SW_OPERATION_ENABLED_MASK  0x27F
//beginning adress of the different variables in the RxPDO
#define ControlWorld           	0
#define TargetPosition          2
#define TargetSpeed          6
#define TargetTorque            10
#define OpModeRx	        12
//beginning adress of the different variables in the RxPDO
#define StatusWord          	0
#define PositionActualValue     2
#define SpeedActualValue     6
#define TorqueActualValue       10
#define OpModeTx	        12

using namespace std;

namespace servos
{
class SoemSGDV: public SoemDriver
{
public:
    SoemSGDV (ec_slavet* mem_loc);
    ~SoemSGDV();
    
    bool configure();
//      void update();
    
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
    
    vector <parameter> m_params;

};
}
#endif