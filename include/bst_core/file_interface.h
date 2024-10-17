#ifndef _FILE_INTERFACE_H_
#define _FILE_INTERFACE_H_

#include <stdint.h>
#include <string>

#include "comm_interface.h"

class FileInterface : public CommunicationsInterface {
	public:
		FileInterface();
		~FileInterface();

		bool initialize(const char * filename, const char * = NULL, const char * = NULL);

		bool open();
		bool close();

		int16_t read(uint8_t * buf, uint16_t buf_size);
		int16_t write(uint8_t * buf, uint16_t size);

		uint16_t getRxBytes() {return 0;}
		uint16_t getTxBytes() {return 0;}

	private:
		int in_fid = -1;

		std::string file_str;
};

#endif
