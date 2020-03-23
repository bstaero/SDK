#ifndef _BST_MODULE_HWIL_H_
#define _BST_MODULE_HWIL_H_

#include "bst_protocol.h"

using namespace bst::comms;

class BSTModuleHWIL : public BSTCommunicationsModule {
	public:
		BSTModuleHWIL();

		void parse(uint8_t type, uint8_t action, uint8_t * data, uint16_t size);
};

#endif
