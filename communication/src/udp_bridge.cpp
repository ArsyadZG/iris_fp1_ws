#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <thread>

namespace {
    constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
    constexpr size_t kPacketSize = 20;

    struct Feedback {
        int32_t left_ticks; int32_t right_ticks;
        int32_t vel_x_cms; int32_t vel_y_cms;
    };

    int32_t read_i32_le(const uint8_t *src) {
        uint32_t raw;
        std::memcpy(&raw, src, sizeof(raw));
        return static_cast<int32_t>(le32toh(raw));
    }

    bool parse_packet(const uint8_t *buf, size_t len, Feedback &out) {
        if (len < kPacketSize || std::memcmp(buf, kHeader, sizeof(kHeader)) != 0) return false;
        out.left_ticks = read_i32_le(buf + 4);
        out.right_ticks = read_i32_le(buf + 8);
        out.vel_x_cms = read_i32_le(buf + 12);
        out.vel_y_cms = read_i32_le(buf + 16);
        return true;
    }
} // namespace

