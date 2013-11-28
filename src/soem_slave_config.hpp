#ifndef SLAVE_CONFIG_H
#define SLAVE_CONFIG_H

#include <stdint.h>
#include <iostream>
#include <sstream>

// //Mask for StatusWord comparison
// #define SW_NOT_READY_SWICH_ON_MASK 0x4F
// #define SW_SWITCH_ON_DISABLED_MASK 0x4F
// #define SW_READY_SWITCH_ON_MASK    0x6F
// #define SW_SWITCHED_ON_MASK        0x27F
// #define SW_OPERATION_ENABLED_MASK  0x27F

template<class T>
inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
{
    std::stringstream ss;
    ss << f << t;
    return ss.str();
};

typedef struct
{
  uint16_t   index;
  uint8_t    subindex;
  uint8_t    size;
  int      param;
  std::string   name;
  std::string   description;
} parameter;

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

#endif