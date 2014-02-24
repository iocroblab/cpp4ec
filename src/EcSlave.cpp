#include "EcSlave.h"
#include "EcUtil.h"

namespace cpp4ec
{
  EcSlave::EcSlave(ec_slavet* mem_loc) : m_datap(mem_loc), m_name("Slave_" + to_string(m_datap->configadr,
				     std::hex)), m_slave_nr(m_datap->configadr & 0x0f){}

  EcSlave::~EcSlave(){}

  const std::string& EcSlave::getName() const
  {
    return m_name;
  }

  bool EcSlave::configure()
  {
    return true;
  }
  
  std::vector<char*> EcSlave::start(){}
  
  std::vector<char*> EcSlave::stop(){}
  
  void EcSlave::setPDOBuffer(char * input, char * output){}

  bool EcSlave::requestState( ec_state state)
  {
    m_datap->state = state;
    ec_writestate(m_slave_nr);
    ec_statecheck(m_slave_nr,state,EC_TIMEOUTSTATE);
    return m_datap->state == state;

  }

  bool EcSlave::checkState( ec_state state)
  {
    ec_statecheck(m_slave_nr,state,EC_TIMEOUTSTATE);
    return m_datap->state == state;

  }

  ec_state EcSlave::getState()
  {
    return (ec_state)(m_datap->state);
  }  

}