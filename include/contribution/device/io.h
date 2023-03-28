#ifndef COMMON_INTERFACE_IO_HW_H
#define COMMON_INTERFACE_IO_HW_H

#include <vector>
#include <string>
#include <memory>

namespace arcs {
namespace common_interface {

/// Io设备
class IoHardware
{
public:
    /**
     * IO寻址策略: 二级寻址, 32bits
     *  偏移            端口号    实际索引
     * (OFFSET) << 16 + PORT  = INDEX
     * IO硬件的典型偏移值
     */
    enum Offset : uint16_t
    {
        Default = 0x0000,      ///< (默认)接口板
        Configurable = 0x0001, ///< 可配置IO
        StaticSafety = 0x0002, ///< 固定的 安全IO
        StaticLink = 0x0003,   ///< 固定的 联动IO
        Tool = 0x0004,         ///< 工具端
        Pedestal = 0x0010,     ///< 底座
        Joint1 = 0x0011,       ///< 关节
        Joint2 = 0x0012,       ///< 关节
        Joint3 = 0x0013,       ///< 关节
        Joint4 = 0x0014,       ///< 关节
        Joint5 = 0x0015,       ///< 关节
        Joint6 = 0x0016,       ///< 关节，可向下扩展
        External = 0x0020,     ///< 外扩IO
        User = 0x1000,         ///< 外扩IO
    };

    virtual ~IoHardware() = default;

    virtual uint16_t getOffset() = 0;

    virtual int getDigitalInputNum() const = 0;
    virtual int getDigitalOutputNum() const = 0;
    virtual int getAnalogInputNum() const = 0;
    virtual int getAnalogOutputNum() const = 0;

    virtual std::vector<bool> getDigitalInputs() const = 0;
    virtual std::vector<bool> getDigitalOutputs() const = 0;
    virtual int setDigitalOutput(int port, bool value) = 0;

    virtual std::vector<double> getAnalogInputs() const = 0;
    virtual std::vector<double> getAnalogOutputs() const = 0;
    virtual int setAnalogOutput(int port, double value) = 0;

    virtual int setAsDigitalInput(int port, bool input) = 0;
    virtual bool isDigitalInput(int port) = 0;
};
using IoHardwarePtr = std::shared_ptr<IoHardware>;

inline std::string toString(IoHardware::Offset offset)
{
#define E(e)            \
    case IoHardware::e: \
        return #e;

    switch (offset) {
        E(Default);
        E(Configurable);
        E(StaticSafety);
        E(StaticLink);
        E(Tool);
        E(Pedestal);
        E(Joint1);
        E(Joint2);
        E(Joint3);
        E(Joint4);
        E(Joint5);
        E(Joint6);
        E(External);
        E(User);
    default:
        if (offset > IoHardware::User) {
            return std::string("User") +
                   std::to_string(offset - IoHardware::User);
        }
    }
#undef E
}

inline std::ostream &operator<<(std::ostream &os, IoHardware::Offset v)
{
    os << toString(v);
    return os;
}

} // namespace common_interface
} // namespace arcs
#endif // COMMON_INTERFACE_IO_HW_H
