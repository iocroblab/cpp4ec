#ifndef MASTER_ERROR_HPP
#define MASTER_ERROR_HPP

#include <exception>
#include "../soem_slave_config.hpp"

using namespace std;


class MasterError: public exception
{
  
public:
  
  enum OwnErrorCode{
    ECAT_ERROR,
    FAIL_SWITCHING_STATE_INIT,
    FAIL_SWITCHING_STATE_PRE_OP,
    FAIL_SWITCHING_STATE_SAFE_OP,
    FAIL_SWITCHING_STATE_OPERATIONAL,
    FAIL_CREATING_DRIVER,
  };
  
  MasterError(int errorcode) throw();
  const char* what() const throw();
  int getErrorCode() const throw ();
  
private:
  int m_errorcode;
  
};

#endif