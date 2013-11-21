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

//#include "servos_rt.h"
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
    
    vector <parameter> m_params;

};
}
#endif