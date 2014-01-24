#ifndef ECUTIL_H
#define ECUTIL_H

#include <stdint.h>

namespace cpp4ec
{
   
   char* dtype2string(uint16_t dtype);
   char* SDO2string(uint16_t slave, uint16_t index, uint8_t subidx, uint16_t dtype);
   
   template<class T>
   inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
   {
      std::stringstream ss;
      ss << f << t;
      return ss.str();
   };
   
   
}


#endif