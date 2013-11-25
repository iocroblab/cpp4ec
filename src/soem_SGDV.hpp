#ifndef SOEM_SGDV_HPP
#define SOEM_SGDV_HPP

#include "soem_slave.hpp"
#include "./errors/SGDV_error.hpp"

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

#include <vector>
#include <iostream>

using namespace std;

namespace servos
{
class SoemSGDV: public SoemDriver
{
public:
    SoemSGDV (ec_slavet* mem_loc);
    ~SoemSGDV();
    
    bool configure() throw(SGDVError);
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