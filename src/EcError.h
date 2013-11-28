#ifndef ECERROR_H
#define ECERROR_H

#include <exception>
#include <iostream>
#include <string>
#include <sstream>


class EcError/*: public exception*/
{

public:

  enum Ec4CppErrorCode{
    ECAT_ERROR,
    FAIL_SWITCHING_STATE_INIT,
    FAIL_SWITCHING_STATE_PRE_OP,
    FAIL_SWITCHING_STATE_SAFE_OP,
    FAIL_SWITCHING_STATE_OPERATIONAL,
    FAIL_CREATING_DRIVER,
  };

  EcError(Ec4CppErrorCode errorcode) throw();
  const char* what() const throw();
  Ec4CppErrorCode getErrorCode() const throw ();

private:
  int m_errorcode;

};

template<class T>
inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
{
   std::stringstream ss;
   ss << f << t;
   return ss.str();
};

#endif