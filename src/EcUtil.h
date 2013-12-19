#ifndef ECUTIL_H
#define ECUTIL_H


namespace cpp4ec
{
   
   char* dtype2string(uint16 dtype);
   char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype);
   
   template<class T>
   inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
   {
      std::stringstream ss;
      ss << f << t;
      return ss.str();
   };
   
   
}


#endif