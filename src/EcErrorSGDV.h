#ifndef ECERRORSGDV_H
#define ECERRORSGDV_H

#include "EcError.h"


class EcErrorSGDV: public EcError
{
public:
//    enum Ec4CppErrorCode{
//     ECAT_ERROR,
//     FAIL_SWITCHING_STATE_INIT,
//     FAIL_SWITCHING_STATE_PRE_OP,
//     FAIL_SWITCHING_STATE_SAFE_OP,
//     FAIL_SWITCHING_STATE_OPERATIONAL,
//     FAIL_CREATING_DRIVER,
//     XML_STRUCTURE_ERROR,
//     XML_NOT_FOUND_ERROR,
//     };
   inline EcErrorSGDV( Ec4CppErrorCode errorcode):EcError( errorcode)
  {   
  }
};

#endif