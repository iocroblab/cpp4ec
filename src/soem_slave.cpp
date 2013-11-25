#include "soem_slave.hpp"


namespace servos
{
  SoemDriver::SoemDriver(ec_slavet* mem_loc) : m_datap(mem_loc), m_name("Slave_" + to_string(m_datap->configadr,
				     std::hex)), m_slave_nr(m_datap->configadr & 0x0f)
  {   
    
  }
				     
  SoemDriver::~SoemDriver()
  {
  }

  const std::string& SoemDriver::getName() const
  {
    return m_name;
  }

  bool SoemDriver::configure()
  {
    return true;
  }

  bool SoemDriver::requestState( ec_state state)
  {
    m_datap->state = state;
    ec_writestate(m_slave_nr);
    ec_statecheck(m_slave_nr,state,EC_TIMEOUTSTATE);
    return m_datap->state == state;
    
  }

  bool SoemDriver::checkState( ec_state state)
  {
    ec_statecheck(m_slave_nr,state,EC_TIMEOUTSTATE);
    return m_datap->state == state;
    
  }

  ec_state SoemDriver::getState()
  {
    return (ec_state)(m_datap->state);
  }
}