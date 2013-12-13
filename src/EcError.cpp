#include "EcError.h"

extern "C"
{
#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatprint.h>
}


EcError::EcError(int errorcode) throw ():m_errorcode(errorcode)
{
}

int EcError::getErrorCode() const throw ()
{
  return m_errorcode;
}


const char* EcError::what() const throw()
{
  int state;

   switch(m_errorcode)
   {
     case ECAT_ERROR:
       return ec_elist2string();
       break;

     case   FAIL_SWITCHING_STATE_INIT:
       state=EC_STATE_INIT;
       for (int i = 1; i <= ec_slavecount; i++)
       {
	 if (ec_slave[i].state != state)
	 {
	  std::cout << "Slave " << i
	       << " State= " << to_string(ec_slave[i].state, std::hex)
	       << " StatusCode=" << ec_slave[i].ALstatuscode
	       << " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode)
	       << std::endl;
	 }
       }
       return std::string("Error: Failed switching to INIT state.").c_str();
       break;

     case FAIL_SWITCHING_STATE_PRE_OP:
       state=EC_STATE_PRE_OP;
       for (int i = 1; i <= ec_slavecount; i++)
       {
	 if (ec_slave[i].state != state)
	 {
	  std::cout << "Slave " << i
	       << " State= " << to_string(ec_slave[i].state, std::hex)
	       << " StatusCode=" << ec_slave[i].ALstatuscode
	       << " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode)
	       << std::endl;
	 }
       }
       return std::string("Error: Failed switching to PRE_OPERATIONAL state.").c_str();
       break;

     case FAIL_SWITCHING_STATE_SAFE_OP:
       state=EC_STATE_SAFE_OP;
       for (int i = 1; i <= ec_slavecount; i++)
       {
	 if (ec_slave[i].state != state)
	 {
	  std::cout << "Slave " << i
	       << " State= " << to_string(ec_slave[i].state, std::hex)
	       << " StatusCode=" << ec_slave[i].ALstatuscode
	       << " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode)
	       << std::endl;
	 }
       }
       return std::string("Error: Failed switching to SAFE_OPERATIONAL state.").c_str();
       break;

     case FAIL_SWITCHING_STATE_OPERATIONAL:
       state=EC_STATE_OPERATIONAL;
       for (int i = 1; i <= ec_slavecount; i++)
       {
	 if (ec_slave[i].state != state)
	 {
	  std::cout << "Slave " << i
	       << " State= " << to_string(ec_slave[i].state, std::hex)
	       << " StatusCode=" << ec_slave[i].ALstatuscode
	       << " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode)
	       << std::endl;
	 }
       }
       return std::string("Error: Failed switching to OPERATIONAL state.").c_str();
       break;

     case FAIL_CREATING_DRIVER:

       return std::string("Error: Failed creating driver").c_str();
       break;

     default:
       return std::string("Error: Invalid error code or unexpected error.").c_str();
       break;
   }

}
