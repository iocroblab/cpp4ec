#ifndef ECSLAVE_H
#define ECSLAVE_H

#include <iostream>
#include <sstream>
#include "EcError.h"
//#include "soem_slave_config.hpp"

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



// template<class T>
// inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
// {
//     std::stringstream ss;
//     ss << f << t;
//     return ss.str();
// };

#include <stdint.h>

// //Mask for StatusWord comparison
// #define SW_NOT_READY_SWICH_ON_MASK 0x4F
// #define SW_SWITCH_ON_DISABLED_MASK 0x4F
// #define SW_READY_SWITCH_ON_MASK    0x6F
// #define SW_SWITCHED_ON_MASK        0x27F
// #define SW_OPERATION_ENABLED_MASK  0x27F


typedef struct
{
   uint16_t   index;
   uint8_t    subindex;
   uint8_t    size;
   int      param;
   std::string   name;
   std::string   description;
} parameter;


namespace ec4cpp
{
class EcSlave
{
public:
    virtual ~EcSlave();

    const std::string& getName() const;

    
    virtual bool configure();
    virtual void start()=0;
    virtual void stop();

    virtual bool requestState( ec_state state);
    virtual bool checkState( ec_state state);
    virtual ec_state getState();

protected:
    EcSlave(ec_slavet* mem_loc);

    ec_slavet* m_datap;
    std::string m_name;
    unsigned int m_slave_nr;
};
}
#endif