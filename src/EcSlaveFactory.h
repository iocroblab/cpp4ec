/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:26:15 CET 2010  soem_driver_factory.cpp

 soem_driver_factory.h -  description
 -------------------
 begin                : Tue November 16 2010
 copyright            : (C) 2010 Ruben Smits
 email                : first.last@mech.kuleuven.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef ECSLAVEFACTORY_H
#define ECSLAVEFACTORY_H

#include <map>
#include <string>
#include "EcSlave.h"

namespace cpp4ec
{
/**
* \brief Class EcSlaveFactory
* 
* This class create an instance of the slave classes detected on the net.   
*/  
class EcSlaveFactory
{
public:
    /**
     * \brief Makes an instance of the class
     */
    static EcSlaveFactory& Instance()
    {
        static EcSlaveFactory soem_driver_factory;
        return soem_driver_factory;
    }
    
    /**
     * \brief A callback function
     */
    typedef EcSlave* (*CreateDriverCallBack)(ec_slavet*);
    
    
    /**
     * \brief Register the driver
     */
    bool registerDriver(std::string name, CreateDriverCallBack createFn);
    
    /**
     * \brief Create a driver
     */
    EcSlave* createDriver(ec_slavet* mem_loc);
    
    /**
     * \brief Display the drivers
     */
    void displayAvailableDrivers();

private:
    typedef std::map<std::string, CreateDriverCallBack> FactoryMap;
    FactoryMap m_factory_map;

    EcSlaveFactory()
    {
    }
    ;
    EcSlaveFactory(const EcSlaveFactory&);
    EcSlaveFactory& operator=(const EcSlaveFactory&);
    ~EcSlaveFactory()
    {
    }
    ;
};
}
#endif
