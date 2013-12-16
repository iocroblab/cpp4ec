#include "EcErrorSGDV.h"

#include <stdio.h>
#include <string.h>

EcErrorSGDV::EcErrorSGDV (int errorcode, unsigned int nslave, std::string name): EcError( errorcode),slave_nr(nslave),slave_name(name)
{  
  
}

const char* EcErrorSGDV::what() const throw()
{
   const char * errormessage;
//   const char * error;
  switch(getErrorCode())
   {
    case ECAT_ERROR:
    case FAIL_SWITCHING_STATE_INIT:
    case FAIL_SWITCHING_STATE_PRE_OP:
    case FAIL_SWITCHING_STATE_SAFE_OP:
    case FAIL_SWITCHING_STATE_OPERATIONAL:
    case FAIL_CREATING_DRIVER:
      
      errormessage = EcError::what();
      return (std::string(errormessage)+slave_name).c_str();
      break;
    case XML_STRUCTURE_ERROR:
      return (std::string("Error: Wrong XML structure. ")+slave_name).c_str();
      break;
    case XML_NOT_FOUND_ERROR:
      return (std::string("Error: XML file not found. ")+slave_name).c_str();
      break;
    case XML_TYPE_ERROR:
      return (std::string("Error: XML wrong types found. ")+slave_name).c_str();
      break;
    default:
      return (std::string("Error: Invalid error code or unexpected error. ")+slave_name).c_str();
      break;
   }
}