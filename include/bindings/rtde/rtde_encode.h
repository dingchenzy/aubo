#ifndef COMMON_INTERFACE_RTDE_ENCODE_H
#define COMMON_INTERFACE_RTDE_ENCODE_H

#include <stdint.h>
#if defined(__linux__) || defined(__linux)
#include <arpa/inet.h> // htonl, ntohl
#elif defined(_WIN32)
#include <windows.h>
#endif
#include <string>
#include <vector>
#include <functional>
#include <iostream>

namespace arcs {
// struct RTDETransportFormat __attribute__ ((__packed__))
// {
//   char      sof[3] = {'S', 'O', 'F'};
//   char      version;
//   uint16_t  len;
// }

const int kHeaderLen = 6;

template <typename T, typename S>
inline void encode(char version, const T &message, S &output)
{
    int32_t len = static_cast<int32_t>(message.size());
    uint8_t a = (len >> 8) & 0xFF;
    uint8_t b = len & 0xFF;

    output.reserve(kHeaderLen + len);
    output.push_back('S');
    output.push_back('O');
    output.push_back('F');
    output.push_back(version);
    output.push_back(a);
    output.push_back(b);
    std::copy(message.begin(), message.end(), std::back_inserter(output));

    for (size_t i = output.size(); i < (kHeaderLen + len); i++) {
        output.push_back(0);
    }
}

template <typename T>
inline uint16_t decode(char version, const T &message)
{
    if (message.size() != kHeaderLen) {
        return 0xFFFF;
    }
    if (message.at(0) != 'S') {
        return 0xFFFF;
    }
    if (message.at(1) != 'O') {
        return 0xFFFF;
    }
    if (message.at(2) != 'F') {
        return 0xFFFF;
    }
    if (message.at(3) != version) {
        return 0xFFFF;
    }

    uint8_t a = message.at(4);
    uint8_t b = message.at(5);
    return ((((uint16_t)a) << 8) & 0xFF00) + (((uint16_t)b) & 0x00FF);
}

template <typename T>
inline uint16_t decode1(char version, const T &message)
{
    if (message.size() != kHeaderLen - 3) {
        return 0xFFFF;
    }
    if (message.at(0) != version) {
        return 0xFFFF;
    }

    uint8_t a = message.at(1);
    uint8_t b = message.at(2);
    return ((((uint16_t)a) << 8) & 0xFF00) + (((uint16_t)b) & 0x00FF);
}

class PackageDispatch
{
public:
    PackageDispatch(std::function<int(char *, size_t)> func = {}) : pkg_cb(func)
    {
    }

    int dispatch(char *data, size_t sz,
                 std::function<int(char *, size_t)> func = {})
    {
        int retval = 0;
        size_t index = 0;
        while (index < sz) {
            switch (stat) {
            case HEAD_S:
                if (data[index++] == 'S') {
                    stat = HEAD_O;
                } else {
                    retval = -1;
                }
                break;
            case HEAD_O:
                if (data[index++] == 'O') {
                    stat = HEAD_F;
                } else {
                    stat = HEAD_S;
                    retval = -1;
                }
                break;
            case HEAD_F:
                if (data[index++] == 'F') {
                    stat = HEAD_VERSION;
                } else {
                    stat = HEAD_S;
                    retval = -1;
                }
                break;
            case HEAD_VERSION:
                version = data[index++];
                stat = HEAD_LEN_H;
                break;
            case HEAD_LEN_H:
                len = (((uint16_t)data[index++]) << 8) & 0xFF00;
                stat = HEAD_LEN_L;
                break;
            case HEAD_LEN_L: {
                len += (((uint16_t)data[index++])) & 0x00FF;
                package.reserve(len);
                stat = BODY;
            } break;
            case BODY:
                if ((sz - index) < (len - package.size())) {
                    std::copy(data + index, data + sz,
                              std::back_inserter(package));
                    index = sz;
                } else {
                    std::copy(data + index,
                              data + index + (len - package.size()),
                              std::back_inserter(package));
                    index += (len - package.size());

                    // 形成了完整的包
                    if (func) {
                        func(package.data(), len);
                    } else if (pkg_cb) {
                        pkg_cb(package.data(), len);
                    }
                    stat = HEAD_S;
                    len = 0;
                    package.clear();
                }
                break;
            }
        }
        return retval;
    }

    void reset()
    {
        stat = HEAD_S;
        len = 0;
        version = 0;
        package.clear();
    }

private:
    enum State
    {
        HEAD_S,
        HEAD_O,
        HEAD_F,
        HEAD_VERSION,
        HEAD_LEN_H,
        HEAD_LEN_L,
        BODY
    };

    std::function<int(char *, size_t)> pkg_cb;

    State stat{ HEAD_S };
    char version{ 0 };
    uint16_t len{ 0 };

    std::vector<char> package;
};

} // namespace arcs

#endif
