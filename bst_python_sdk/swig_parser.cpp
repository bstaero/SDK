#include "swig_parser.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <unordered_set>

// Packet types to parse when quick mode is enabled
std::unordered_set<int> quick_mode_packets = {
    1,   // SENSORS_GPS
    16,  // STATE_STATE
    33,  // CONTROL_COMMAND
    81,  // SYSTEM_INITIALIZE
};

std::vector<Packet> parse(const char* file_path, bool has_addr, bool quick_mode) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return {};
    }

    file.seekg(0, std::ios::end);
    std::streamsize length = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> buf(length);
    if (!file.read(reinterpret_cast<char*>(buf.data()), length)) {
        std::cerr << "Failed to read file: " << file_path << std::endl;
        return {};
    }

    std::vector<Packet> packets;
    std::size_t i = 0;
    while (i < buf.size()) {
        if (buf[i] == 'U' && buf[i + 1] == '$') {
            int buf_start = i;
            i += 2;

            Packet packet;
            std::vector<uint8_t> packet_data;

            packet.TYPE = buf[i++];

            // In quick mode, only parse SYS_INIT and CONTROL_COMMAND
            if (quick_mode && !quick_mode_packets.count(packet.TYPE)) {
                continue;
            }

            packet.ACTION = buf[i++];
            packet.SIZE = buf[i] | (buf[i + 1] << 8);
            i += 2;

            if (has_addr) {
                std::memcpy(&packet.TO, buf.data() + i, sizeof(int));
                i += 4;
                std::memcpy(&packet.FROM, buf.data() + i, sizeof(int));
                i += 4;
            }

            if (i + packet.SIZE > buf.size()) {
                std::cerr << "Malformed packet: size exceeds buffer length" << std::endl;
                break;
            }

            packet.DATA.assign(buf.begin() + i, buf.begin() + i + packet.SIZE);
            i += packet.SIZE;

            if (quick_mode && packet.TYPE == 33) {
                if (packet.DATA.empty() || packet.DATA.at(0) != 1) { // Skip non-flight mode commands
                    continue;
                }
            }

            if (i + 2 <= buf.size()) {
                packet.CHKSUM.assign(buf.begin() + i, buf.begin() + i + 2);
                i += 2;
            }

            int buf_len = i - buf_start;
            packet_data.assign(buf.begin() + buf_start, buf.begin() + buf_start + buf_len);

            if (check_fletcher_16(packet_data, buf_len)) {
                packets.push_back(packet);
            }
        } else {
            i++;
        }
    }

    return packets;
}

bool check_fletcher_16(const std::vector<uint8_t>& data, int data_size) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;

    for (int i = 0; i < data_size; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    uint16_t checksum = (sum2 << 8) | sum1;
    return checksum == 0;
}

