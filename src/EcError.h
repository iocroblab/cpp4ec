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
    FAIL_SOCKET_OUTPUT,
    FAIL_SOCKET_INPUT,
    FAIL_SETSOCKOPT_OUTPUT,
    FAIL_SETSOCKOPT_INPUT,
    FAIL_BINDING,
    FAIL_CONNECTING,
    FAIL_GETTING_PEERNAME_INPUT,
    FAIL_RECIEVING,
    FAIL_SENDING,
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