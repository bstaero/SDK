#include <stdio.h>
#include "bst_packet.h"
#include "helper_functions.h"
#include "debug.h"

uint16_t Packet_validateBuffer(const uint8_t * const data, uint16_t n)
{
	// now make sure there are enough bytes to hold a minimal packet
	if( n == 0 || n < OVERHEAD(true) )
		return BUFF_NEED_MORE;

	// check for packet start
	if( data[0] != 'U' || data[1] != '$' )
		return BUFF_INVALID;

	// now get data and packet size from buffer
	uint16_t p_size = Packet_packetSize(data);

	// check for valid sizes
	if( p_size > BST_MAX_PACKET_SIZE )
		return BUFF_INVALID;

	if( n < p_size )
		return BUFF_NEED_MORE;

	// now check checksum
	if(checkFletcher16(data, p_size))
		return p_size;

	return BUFF_INVALID;
}

uint16_t Packet_packetSize(const uint8_t * const data)
{
	return Packet_dataSize(data) + OVERHEAD(true);
}

uint16_t Packet_dataSize(const uint8_t * const data)
{
	return (uint16_t)(data[PKT_SIZE+1] << 8) | (uint16_t)(data[PKT_SIZE]);
}

Packet::Packet() {
	// do big_endian check
	uint16_t temp = 0x0100;
	big_endian = ((uint8_t *)&temp)[0];

	// setup addressing
	uses_address = true;

	// initialize
	initialize();
}

Packet::Packet(Packet_t type) {

	// do big_endian check
	uint16_t temp = 0x0100;
	big_endian = ((uint8_t *)&temp)[0];

	// setup addressing
	uses_address = true;

	// initialize
	initialize();

	// set packet type
	packet[PKT_TYPE] = type;
}

Packet::~Packet(void) { }

void Packet::initialize() {

	// zero buffer
	memset( packet, 0, BST_MAX_PACKET_SIZE); 

	// fill with default values
	packet[PKT_HDR0]   = 'U';
	packet[PKT_HDR1]   = '$';
	packet[PKT_TYPE]   = INVALID_PACKET;
	packet[PKT_ACTION] = PKT_ACTION_STATUS;
	packet[PKT_SIZE]   = 0;
	packet[PKT_SIZE+1] = 0;

	if( uses_address ) {
		setFromAddress(NO_ID);
		setToAddress(ALL_NODES);
	}

	ptr = 0;
}

uint32_t Packet::getToAddress (void) const {
	uint32_t address = NO_ID;
	if( uses_address ) 
		memcpy((uint8_t*)&address,(uint8_t*)&(packet[PKT_TO]),4);

	if(big_endian) address = changeEndiannessUint32(address);

	return address;
}

void Packet::setToAddress(uint32_t id) {
	uses_address = true;
	if(big_endian) id = changeEndiannessUint32(id);

	memcpy((uint8_t*)&packet[PKT_TO],(uint8_t*)&id,4);
}

bool Packet::isToID (uint32_t mask) const {

	uint32_t addr = getToAddress();

	// its to no one
	if( addr == NO_ID || mask == NO_ID )
		return false;

	// simple case: its to me or to all nodes
	if( mask == addr || mask == ALL_NODES || addr == ALL_NODES || addr == ONE_HOP) 
		return true;

	// check that the node types match, that is they 
	//   - directly match
	//   - or one of them specifies all
	uint32_t my_type = addr & NODE_TYPE_MASK;
	uint32_t mask_type = mask & NODE_TYPE_MASK;
	
	if( my_type == mask_type || mask_type == NODE_TYPE_MASK || my_type == NODE_TYPE_MASK ) {

		// now check serial numbers match, that is they
		//   - directly match
		//   - or one of them specifies all
		uint32_t my_num = addr & NODE_SERIAL_MASK;
		uint32_t mask_num = mask & NODE_SERIAL_MASK;
		
		if( my_num == mask_num || mask_num == NODE_SERIAL_MASK || my_num == NODE_SERIAL_MASK ) 
			return true;
	}

	return false;
}

bool Packet::isFromID (uint32_t mask) const {

	if(big_endian) mask = changeEndiannessUint32(mask);

	uint32_t addr = getFromAddress();

	// its to no one
	if( addr == NO_ID || mask == NO_ID )
		return false;

	// simple case: its to me or to all nodes
	if( mask == addr || mask == ALL_NODES || addr == ALL_NODES || addr == ONE_HOP) 
		return true;

	// check that the node types match, that is they 
	//   - directly match
	//   - or one of them specifies all
	uint32_t my_type = addr & NODE_TYPE_MASK;
	uint32_t mask_type = mask & NODE_TYPE_MASK;
	
	if( my_type == mask_type || mask_type == NODE_TYPE_MASK || my_type == NODE_TYPE_MASK ) {

		// now check serial numbers match, that is they
		//   - directly match
		//   - or one of them specifies all
		uint32_t my_num = addr & NODE_SERIAL_MASK;
		uint32_t mask_num = mask & NODE_SERIAL_MASK;
		
		if( my_num == mask_num || mask_num == NODE_SERIAL_MASK || my_num == NODE_SERIAL_MASK ) 
			return true;
	}

	return false;
}


uint32_t Packet::getFromAddress (void) const {
	uint32_t address = NO_ID;
	if( uses_address ) 
		memcpy((uint8_t*)&address,(uint8_t*)&(packet[PKT_FROM]),4);

	if(big_endian) address = changeEndiannessUint32(address);

	return address;
}

void Packet::setFromAddress(uint32_t id) {
	uses_address = true;

	if(big_endian) id = changeEndiannessUint32(id);

	memcpy((uint8_t*)&packet[PKT_FROM],(uint8_t*)&id,4);
}


uint16_t Packet::getSize(void) const {
	return (this->getDataSize() + OVERHEAD(uses_address));
}

uint16_t Packet::getDataSize (void) const { 
	return ((uint16_t)(packet[PKT_SIZE+1] << 8) | (uint16_t)(packet[PKT_SIZE]));
}

void Packet::setSize(uint16_t size) { 
   packet[PKT_SIZE+1] = size/256;
   packet[PKT_SIZE] = size%256;
}

Packet_t Packet::getType(void) const { return (Packet_t)(packet[PKT_TYPE]); }
void Packet::setType(Packet_t type) { packet[PKT_TYPE] = type; }

PacketAction_t Packet::getAction(void) const { return (PacketAction_t)(packet[PKT_ACTION]);}
void Packet::setAction(PacketAction_t action) { packet[PKT_ACTION] = action; }


void Packet::setData(uint8_t * data, uint16_t size) {
	memcpy(&packet[PKT_DATA(uses_address)],data,size);
	this->setSize(size);
}

void Packet::getData(uint8_t * data) {
	memcpy(data,&packet[PKT_DATA(uses_address)],this->getDataSize());
}

// search for packet start in packet data
void Packet::resetPacketBuffer()
{ 
	bool packet_start = false;
	uint8_t i=1;
	for( i=1; i < ptr; i++) {
		if( packet_start) {

			// we found the start of another packet
			if( packet[i] == '$' ) {

				pmesg(VERBOSE_WARN, "Packet - dropping bytes=%u\n", i-1);

				//save the memory
				uint8_t n = ptr - i + 1;
				memmove(packet, &packet[i-1], n);

				// reset pointer
				ptr = n;

				return;
			} else
				packet_start = false;
		} else if( packet[i] == 'U' ) {
			packet_start = true;
		}                 
	}                    


	// we only had one byte left
	if( packet_start ) {
		pmesg(VERBOSE_WARN, "Packet - dropping bytes=%u\n", i-1);
		packet[PKT_HDR0] = 'U';
		ptr = PKT_HDR1;
		return;
	}

	// finally, complete reset
	pmesg(VERBOSE_WARN, "Packet - dropping bytes=%u\n", ptr);
	ptr = 0;
}

bool Packet::isValid(uint8_t data) { // Fletcher 16
	switch(ptr) {
		case PKT_HDR0:
			if(data == 'U')
				packet[ptr++] = data;
			break;
		case PKT_HDR1:
			if(data == '$')
				packet[ptr++] = data;
			else
				ptr = 0;
			break;
		default:
			packet[ptr++] = data;

			// see if there are enough bytes to check for minimal packet
			if( ptr >= OVERHEAD(uses_address) ) {

				// check for valid size
				if(this->getSize() > BST_MAX_PACKET_SIZE) {

					resetPacketBuffer();
					return false;
				}

				// now check for enough data
				if( ptr >= this->getSize() ) {

					// check for valid value
					if(checkFletcher16(packet,this->getSize())) {
						// reset buffer as we used all bytes
						ptr = 0;
						return true;
					}

					// reset buffer looking for good bytes
					resetPacketBuffer();
				}
			}
			break;
	}

	return false;
}

bool Packet::setFromBuffer(uint8_t *data, uint16_t n)
{
	if( n <= BST_MAX_PACKET_SIZE )
		memcpy(packet, data, n);
	else 
		return false;

	return true;
}

uint16_t Packet::validateBuffer(uint8_t *data, uint16_t n) 
{
	// now make sure there are enough bytes to hold a minimal packet
	if( n == 0 || n < OVERHEAD(uses_address) )
		return BUFF_NEED_MORE;

	// check for packet start
	if( data[0] != 'U' || data[1] != '$' )
		return BUFF_INVALID;

	// now get data and packet size from buffer
	uint16_t d_size = (uint16_t)(data[PKT_SIZE+1] << 8) | (uint16_t)(data[PKT_SIZE]);
	uint16_t p_size = d_size + OVERHEAD(uses_address);

	// check for valid sizes
	if( p_size > BST_MAX_PACKET_SIZE )
		return BUFF_INVALID;

	if( n < p_size )
		return BUFF_NEED_MORE;

	// now check checksum
	if(checkFletcher16(data, p_size))
		return p_size;

	return BUFF_INVALID;
}

bool Packet::isRequest(void) const {
	return (packet[PKT_ACTION] == PKT_ACTION_REQUEST);
}

void Packet::setCheckSum(void) { // Fletcher 16
	if(this->getSize() >= BST_MAX_PACKET_SIZE) return;
	setFletcher16(packet, this->getSize());
}

uint8_t * Packet::getPacket(void) {
	setCheckSum();
	return packet;
}

void Packet::clear(void) {
	initialize();
}

bool Packet::operator != (const Packet& pkt) const {
	return !(*this == pkt);
}

bool Packet::operator == (const Packet& pkt) const {
	if(packet[PKT_TYPE] == INVALID_PACKET) return false;
	if(this->getSize() != pkt.getSize()) return false;

	if(memcmp((void *)(&packet[PKT_DATA(uses_address)]),(void *)(pkt.getDataPtr()),this->getDataSize()) == 0) {
		return true;
	}

	return false;
}

bool Packet::checkFletcher16(uint8_t * data, uint16_t size) {
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	for( uint16_t i = 0; i < size; i++ ) {
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	return ((sum2 << 8) | sum1) == 0;
}

void Packet::setFletcher16 (uint8_t * data, uint16_t size){
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	for( uint16_t i = 0; i < (size-2); i++ ) {  // excludes checksum bytes
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	data[size-2] = 255 - (( sum1 + sum2 ) % 255);
	data[size-1] = 255 - (( sum1 + data[size-2] ) % 255);
}
