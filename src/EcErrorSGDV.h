#ifndef ECERRORSGDV_H
#define ECERRORSGDV_H

#include "EcError.h"




class EcErrorSGDV: public EcError
{
public:
    enum {
    ECAT_ERROR,
    FAIL_SWITCHING_STATE_INIT,
    FAIL_SWITCHING_STATE_PRE_OP,
    FAIL_SWITCHING_STATE_SAFE_OP,
    FAIL_SWITCHING_STATE_OPERATIONAL,
    FAIL_CREATING_DRIVER,
    XML_STRUCTURE_ERROR,
    XML_NOT_FOUND_ERROR,
    };
    
    EcErrorSGDV (int errorcode, unsigned int nslave, std::string name);
    const char* what() const throw();



private:
  int slave_nr;
  std::string slave_name;
};

#endif