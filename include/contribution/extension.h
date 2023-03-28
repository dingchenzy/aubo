#ifndef COMMON_INTERFACE_EXTENSION_H
#define COMMON_INTERFACE_EXTENSION_H

#include <string>
#include <memory>
#include <aubo/aubo_api.h>
#include <contribution/logging.h>

namespace arcs {
namespace common_interface {

/**
 * 对RobotModule的扩展
 */
class AuboControlExtension
{
public:
    virtual ~AuboControlExtension() = default;

    virtual void start(AuboApiPtr api, LogHandler log_handler,
                       const std::string &config = "") = 0;
    virtual void stop() = 0;

    /**
     * 解析命令行字符串
     *
     * @param argc 参数的数量
     * @param argv 参数
     * @param os 返回的字符串
     * @return int 处理的结果 1: 表示未处理，为默认返回值
     */
    virtual int parseCmdLine(int argc, char **argv, std::ostream &os)
    {
        (void)argc;
        (void)argv;
        (void)os;
        return 1;
    }
};
using AuboControlExtensionPtr = std::shared_ptr<AuboControlExtension>;

} // namespace common_interface
} // namespace arcs
#endif // COMMON_INTERFACE_EXTENSION_H
