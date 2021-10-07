#include "bst_module.h"

#include "debug.h"

//#include <cstddef>

BSTCommunicationsModule::BSTCommunicationsModule() {

	pmesg(VERBOSE_ALLOC, "BSTCommunicationsModule::BSTCommunicationsModule()\n");

	receive_function = NULL;
	receiveCommand_function = NULL;
	receiveReply_function = NULL;
	publish_function = NULL;

	// classes that inherit this class must implement this
	max_num_data_types = 0;
	data_types = NULL;

	// initialize lookup table (LUT)
	for( uint8_t i=0; i < MAX_DATA_TYPES; i++ )
		data_types_lut[i] = INVALID_PACKET;

	num_data_types = 0;

	parent = NULL;
}

BSTCommunicationsModule::~BSTCommunicationsModule () {
	delete[] data_types;
}

void BSTCommunicationsModule::update() {}

void BSTCommunicationsModule::registerReceive(void (* fptr)(uint8_t, void *, uint16_t, const void *)) {
	receive_function = fptr;
}

void BSTCommunicationsModule::registerReceiveCommand(uint8_t (* fptr)(uint8_t, void *, uint16_t, const void *)) {
	receiveCommand_function = fptr;
}

void BSTCommunicationsModule::registerReceiveReply(void (* fptr)(uint8_t, void *, uint16_t, bool, const void *)) {
	receiveReply_function = fptr;
}

void BSTCommunicationsModule::registerPublish(bool (* fptr)(uint8_t, uint8_t)) {
	publish_function = fptr;
}


bool BSTCommunicationsModule::handles(Packet_t type) {
	// check LUT for valid index, indicating valid handler	
	if( data_types_lut[type] != INVALID_PACKET)  {
		return true;
	}

	return false;
} 

void BSTCommunicationsModule::send(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) {
	if(parent == NULL) return;
	parent->write(type,PKT_ACTION_STATUS,data,size,parameter);
}

void BSTCommunicationsModule::sendCommand(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) {
	if(parent == NULL) return;
	parent->write(type,PKT_ACTION_COMMAND,data,size,parameter);
}

void BSTCommunicationsModule::request(uint8_t type, uint8_t parameter){
	if(parent == NULL) return;
	parent->write(type,PKT_ACTION_REQUEST,(uint8_t *)&parameter,1,NULL);
}

void BSTCommunicationsModule::parse(uint8_t type, uint8_t action, uint8_t * data, uint16_t size ) 
{
	// check for valid index
	uint8_t ind = data_types_lut[type];
	uint8_t retval = 0;

	if( ind == INVALID_PACKET) 
		return;

	DataType_t * data_type = &data_types[ind];

	if(action != PKT_ACTION_REQUEST && action != PKT_ACTION_LOAD) {
		// check that the size is what we expect
		if(size != data_type->size) 
			return;

		// fill in data
		memcpy(local_data,data,size);

		switch(action) {
			case PKT_ACTION_COMMAND:
				if(receiveCommand_function == NULL) return;
				if(parent == NULL) return;
				if(!data_type->command) return;

				retval = receiveCommand_function(type,local_data,size,(void*)NULL);

				if(retval == 1) {
					parent->write(type,PKT_ACTION_ACK,data,size,NULL);
				} else {
					// Delayed ACK -- don't automatically reply
					if(retval == 2) {
						// TODO Nothing yet
					} else {
						parent->write(type,PKT_ACTION_NACK,data,size,NULL);
					}
				}
				break;

			case PKT_ACTION_ACK:
				if(receiveReply_function == NULL) return;
				if(!data_type->command) return;
				receiveReply_function(type,data,size,true,(void*)NULL);
				break;

			case PKT_ACTION_NACK:
				if(receiveReply_function == NULL) return;
				if(!data_type->command) return;
				receiveReply_function(type,data,size,false,(void*)NULL);
				break;

			case PKT_ACTION_STATUS:
				if(receive_function == NULL) return;
				receive_function(type,local_data,size,(void*)NULL);
				break;
		}
	} else {
		if (!data_type->request) return;
		if (!publish_function(type,data[0])) {
			parent->write(type,PKT_ACTION_NACK,data,size,NULL);
		}
	}
}                                   


void BSTCommunicationsModule::registerDataType(uint8_t type, uint16_t size, bool command, bool request) {
	if(num_data_types < max_num_data_types) {

		// store index to look-up
		data_types_lut[type] = num_data_types;

		// fill in data type struct for handler
		data_types[num_data_types].size = size;
		data_types[num_data_types].command = command;
		data_types[num_data_types].request = request;

		// increment number of registered handlers
		num_data_types++;
	}
}

void BSTCommunicationsModule::setProtocol(BSTProtocol * a_protocol) {
	parent = a_protocol;
}
