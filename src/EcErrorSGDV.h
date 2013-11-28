#ifndef ECERRORSGDV_H
#define ECERRORSGDV_H

#include "EcError.h"


class EcErrorSGDV: public EcError
{
public:
   inline EcErrorSGDV( Ec4C++ErrorCode errorcode):EcError( Ec4C++ErrorCode)
  {
  }
};

#endif