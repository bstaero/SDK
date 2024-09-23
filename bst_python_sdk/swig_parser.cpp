#include "swig_parser.hpp"
#include <fstream>
#include <iostream>
#include <cstring>

std::vector<Packet> parse(const char* file_path, bool has_addr) {
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

