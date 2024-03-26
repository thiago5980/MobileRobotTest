#include <cstdint>
#include <iostream>

int32_t bytesToInt32(const uint8_t* bytes, bool isLittleEndian) {
    if (isLittleEndian) {
        // For little-endian
        return (static_cast<int32_t>(bytes[3]) << 24) |
               (static_cast<int32_t>(bytes[2]) << 16) |
               (static_cast<int32_t>(bytes[1]) << 8)  |
                static_cast<int32_t>(bytes[0]);
    } else {
        // For big-endian
        return (static_cast<int32_t>(bytes[0]) << 24) |
               (static_cast<int32_t>(bytes[1]) << 16) |
               (static_cast<int32_t>(bytes[2]) << 8)  |
                static_cast<int32_t>(bytes[3]);
    }
}

int main() {
    uint8_t data[] = {0x02, 0x03, 0x04, 0x05}; // Example data bytes

    int32_t value = bytesToInt32(data, true); // Change to false if data is big-endian
    std::cout << "The int32_t value is: " << value << std::endl;

    return 0;
}
