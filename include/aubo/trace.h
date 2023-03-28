#ifndef AUBO_SDK_TRACE_INTERFACE_H
#define AUBO_SDK_TRACE_INTERFACE_H

#include <string>
#include <vector>
#include <memory>
#include <sstream>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

/**
 * 提供给控制器扩展程序的日志记录系统
 */
class ARCS_ABI_EXPORT Trace
{
public:
    Trace();
    virtual ~Trace();

    /**
     * 向 aubo_control 日志注入告警信息
     *
     * TraceLevel:
     *  0 - FATAL
     *  1 - ERROR
     *  2 - WARNING
     *  3 - INFO
     *  4 - DEBUG
     *
     * code定义参考 error_stack
     *
     * @param level
     * @param code
     * @param args
     * @return
     *
     * @code Python函数原型
     * alarm(self: pyaubo_sdk.Trace, arg0: arcs::common_interface::TraceLevel,
     * arg1: int, arg2: List[str]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * alarm(level: number, code: number, args: table) -> nil
     * @endcode
     */
    int alarm(TraceLevel level, int code,
              const std::vector<std::string> &args = {});

    /**
     * 打印文本信息到日志中
     *
     * @param msg 文本信息
     * @return
     *
     * @code Python函数原型
     * textmsg(self: pyaubo_sdk.Trace, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * textmsg(msg: string) -> nil
     * @endcode
     */
    int textmsg(const std::string &msg);

    /**
     * 向连接的 RTDE 客户端发送弹窗请求
     *
     * @param level
     * @param title
     * @param msg
     * @param mode
     *   0: 普通模式
     *   1: 阻塞模式
     *   2: 输入模式 bool
     *   3: 输入模式 int
     *   4: 输入模式 double
     *   5: 输入模式 string
     * @return
     *
     * @code Python函数原型
     * popup(self: pyaubo_sdk.Trace, arg0: arcs::common_interface::TraceLevel,
     * arg1: str, arg2: str, arg3: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * popup(level: number, title: string, msg: string, mode: number) -> nil
     * @endcode
     */
    int popup(TraceLevel level, const std::string &title,
              const std::string &msg, int mode);

    /**
     * peek最新的 AlarmInfo(上次一获取之后)
     * last_time设置为0时，可以获取到所有的AlarmInfo
     *
     * @param num
     * @param last_time
     * @return
     *
     * @code Python函数原型
     * peek(self: pyaubo_sdk.Trace, arg0: int, arg1: int) ->
     * List[arcs::common_interface::RobotMsg]
     * @endcode
     *
     * @code Lua函数原型
     * peek(num: number, last_time: number) -> table
     * @endcode
     */
    RobotMsgVector peek(size_t num, uint64_t last_time = 0);

protected:
    void *d_;
};

using TracePtr = std::shared_ptr<Trace>;

// clang-format off
#define Trace_DECLARES                               \
    _INST(Trace, 3, alarm, level, code, args)        \
    _INST(Trace, 4, popup, level, title, msg, mode)  \
    _INST(Trace, 1, textmsg, msg)                    \
    _FUNC(Trace, 2, peek, num, last_time)
// clang-format on
} // namespace common_interface
} // namespace arcs
#endif
