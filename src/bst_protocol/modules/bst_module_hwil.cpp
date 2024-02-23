#include "bst_module_hwil.h"

#include "debug.h"

using namespace bst::comms;

BSTModuleHWIL::BSTModuleHWIL() : BSTCommunicationsModule() {
	pmesg(VERBOSE_ALLOC, "BSTModuleHWIL::BSTModuleHWIL()\n");

	max_num_data_types = MAX_DATA_TYPES;
	data_types = new DataType_t[MAX_DATA_TYPES]; 

	registerDataType(HWIL_CAN, 0, false, false);
	registerDataType(HWIL_SENSORS, 0, false, false);
	registerDataType(HWIL_ACTUATORS, 0, false, false);
}

void BSTModuleHWIL::parse(uint8_t type, uint8_t action, uint8_t * data, uint16_t size ) 
{
	// check for valid index
	uint8_t ind = data_types_lut[type];
	uint8_t retval = 0;

	if( ind == INVALID_PACKET) 
		return;

	switch(type) {

		/* CAN PACKET */
		case HWIL_CAN:

			if(receive_function == NULL) return;

			// fill in data
			memcpy(local_data,data,size);

			receive_function(type,local_data,size,(void*)NULL);

			break;
		}
}                                   
