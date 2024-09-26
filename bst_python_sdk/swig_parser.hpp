#include <vector>
#include <cstdint>

struct Packet {
    int TYPE;
    int ACTION;
    int SIZE;
    std::vector<uint8_t> DATA;
    std::vector<uint8_t> CHKSUM;
    int TO;
    int FROM;
    int PKT_ID;
    int CAN_PKT_SIZE;
};

std::vector<Packet> parse(const char* file_path, bool has_addr, bool quick_mode);
bool check_fletcher_16(const std::vector<uint8_t>& data, int data_size);
