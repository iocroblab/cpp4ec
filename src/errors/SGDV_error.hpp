#ifndef SGDV_ERROR_HPP
#define SGDV_ERROR_HPP

#include "master_error.hpp"
#include "../soem_slave_config.hpp"

using namespace std;


class SGDVError: public MasterError
{
public:
  inline SGDVError(int errorcode):MasterError(errorcode)
  {
  }
};

#endif