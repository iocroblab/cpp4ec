#ifndef SLAVE_CONFIG_H
#define SLAVE_CONFIG_H

#include <stdint.h>


 typedef struct{
	      uint16_t   index;
	      uint8_t    subindex;
	      uint8_t    size;
	      int      param;
	      std::string   name;
	      std::string   description;
	  }parameter;


#endif