#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "file_interface.h"

#include "debug.h"

FileInterface::FileInterface () : CommunicationsInterface() {
	pmesg(VERBOSE_ALLOC, "FileInterface::FileInterface()\n");
}

FileInterface::~FileInterface () {
}

bool FileInterface::initialize(const char * filename, const char * param2, const char * param3) {
	CommunicationsInterface::initialize(filename,NULL,NULL);

	filename ? file_str = filename : file_str = "data.csv";
	comm_type = CommunicationsInterface::LOCAL_FILE;

	return FileInterface::open();

	return false;
}

uint16_t FileInterface::read(uint8_t * buf, uint16_t buf_size) {
	int retval = ::read(in_fid, buf, buf_size);
	if(retval <= 0) connected = false;

	return (uint16_t)retval;
}


uint16_t FileInterface::write(uint8_t * buf, uint16_t buf_size) {
	return 0;
}

bool FileInterface::close() 
{
	::close(in_fid);
	in_fid = -1;
	return true;
}

bool FileInterface::open() {

	if(in_fid != -1) return true;

	in_fid = ::open(file_str.c_str(), O_RDONLY);

	if(in_fid < 0) {
#ifdef DEBUG
		printf("ERROR - unable to open file %s for reading.\n",file_str);
#endif
		return false;
	}

	connected = true;

	return true;
}
