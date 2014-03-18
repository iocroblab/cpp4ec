#ifndef ECERROR_H
#define ECERROR_H

#include <exception>
#include <iostream>
#include <string>
#include <sstream>

class EcError/*: public exception*/
{
public:

  enum 
  {
    ECAT_ERROR,
    FAIL_EC_INIT,
    FAIL_EC_CONFIG_INIT,
    FAIL_SWITCHING_STATE_INIT,
    FAIL_SWITCHING_STATE_PRE_OP,
    FAIL_SWITCHING_STATE_SAFE_OP,
    FAIL_SWITCHING_STATE_OPERATIONAL,
    FAIL_CREATING_DRIVER,    
    FAIL_OPENING_OUTPUT,
    FAIL_WRITING,
    FAIL_OPENING_INPUT,
    FAIL_READING,
  };
  
  /**
  * \brief Constructor
  *   
  */
  EcError(int errorcode) throw();
  
  /**
  * \brief Get string identifying exception
  *   
  */
  virtual const char* what() const throw();
  
  /**
  * \brief Get the code of the error
  *   
  */
  virtual int getErrorCode() const throw ();

private:
  int m_errorcode;

};

#endif