#ifndef ECUTIL_H
#define ECUTIL_H

#include <stdint.h>
#include <string>
#include <iostream>
#include <sstream>



namespace cpp4ec
{

   template<class T>
   inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
   {
      std::stringstream ss;
      ss << f << t;
      return ss.str();
   };
   
   
}


#endif