#ifndef ECERROR_H
#define ECERROR_H

#include <exception>
#include <iostream>
#include <string>
#include <sstream>




class EcError/*: public exception*/
{
public:
  
  enum {
    ECAT_ERROR,
    FAIL_SWITCHING_STATE_INIT,
    FAIL_SWITCHING_STATE_PRE_OP,
    FAIL_SWITCHING_STATE_SAFE_OP,
    FAIL_SWITCHING_STATE_OPERATIONAL,
    FAIL_CREATING_DRIVER,
  };

  EcError(int errorcode) throw();
  virtual const char* what() const throw();
  virtual int getErrorCode() const throw ();

private:
  int m_errorcode;

};

#endif