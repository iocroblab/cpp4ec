#ifndef SOEM_SLAVE_HPP
#define SOEM_SLAVE_HPP

#include <iostream>
#include <sstream>
#include "soem_slave_config.hpp"

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

template<class T>
inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
{
    std::stringstream ss;
    ss << f << t;
    return ss.str();
};

namespace servos
{
class SoemDriver
{
public:
    virtual ~SoemDriver();

    const std::string& getName() const;
    
//     virtual void update()=0;
    virtual bool configure();

    virtual bool requestState( uint16_t state);
    virtual bool checkState( uint16_t state);
    virtual uint16_t getState();

protected:
    SoemDriver(ec_slavet* mem_loc);

    ec_slavet* m_datap;
    std::string m_name;
    unsigned int m_slave_nr;
};
}
#endif