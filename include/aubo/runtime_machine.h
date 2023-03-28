#ifndef AUBO_SDK_RUNTIME_MACHINE_INTERFACE_H
#define AUBO_SDK_RUNTIME_MACHINE_INTERFACE_H

#include <memory>
#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

/**
 * The RuntimeMachine class
 */
class ARCS_ABI_EXPORT RuntimeMachine
{
public:
    RuntimeMachine();
    virtual ~RuntimeMachine();

    /**
     * 设置下一行脚本的运行上下文
     *
     * @param tid 线程ID
     * @param lineno 行号
     * @param comment 注释
     * @return
     *
     * @code Python函数原型
     * setPlanContext(self: pyaubo_sdk.RuntimeMachine, arg0: int, arg1: int,
     * arg2: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setPlanContext(tid: number, lineno: number, comment: string) -> number
     * @endcode
     */
    int setPlanContext(int tid, int lineno, const std::string &comment);

    /**
     * 获取耗时的接口执行状态, 如 setPersistentParameters
     *
     * @return 指令名字, 执行状态
     * 执行状态: EXECUTING/FINISHED
     *
     * @code Python函数原型
     * getExecutionStatus(self: pyaubo_sdk.RuntimeMachine) -> Tuple[str, str]
     * @endcode
     *
     * @code Lua函数原型
     * getExecutionStatus() -> string
     * @endcode
     */
    std::tuple<std::string, std::string> getExecutionStatus();

    /**
     * 跳转到指定行号
     *
     * @param lineno
     * @return
     *
     * @code Python函数原型
     * gotoLine(self: pyaubo_sdk.RuntimeMachine, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * gotoLine(lineno: number) -> number
     * @endcode
     */
    int gotoLine(int lineno);

    /**
     * 获取当前运行上下文
     *
     * @return
     *
     * @code Python函数原型
     * getPlanContext(self: pyaubo_sdk.RuntimeMachine) -> Tuple[int, int, str]
     * @endcode
     *
     * @code Lua函数原型
     * getPlanContext() -> number
     * @endcode
     */
    std::tuple<int, int, std::string> getPlanContext();

    /**
     * 获取提前运行规划器的上下文信息
     *
     * @return
     */
    std::tuple<int, int, std::string> getAdvancePlanContext();

    /**
     * 获取AdvanceRun的程序指针
     *
     * @return
     */
    int getAdvancePtr();

    /**
     * 获取机器人运动的程序指针
     *
     * @return
     */
    int getMainPtr();

    /**
     * 允许在非主线程中执行指令，内部使用
     *
     * @param allowed
     * @return
     *
     * @code Python函数原型
     * setInstructionAllowed(self: pyaubo_sdk.RuntimeMachine, arg0: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setInstructionAllowed(allowed: boolean) -> number
     * @endcode
     */
    int setInstructionAllowed(bool allowed);

    /**
     * 加载本地工程文件
     * Lua 脚本，只需要给出文件名字，不需要后缀，需要从 ${ARCS_WS}/program
     * 目录中查找
     *
     * @param program
     * @return
     */
    int loadProgram(const std::string &program);

    /**
     * 运行已经加载的工程文件
     *
     * @return
     */
    int runProgram();

    /**
     * 开始运行时
     *
     * @return
     *
     * @code Python函数原型
     * start(self: pyaubo_sdk.RuntimeMachine) -> int
     * @endcode
     *
     * @code Lua函数原型
     * start() -> number
     * @endcode
     */
    int start();

    /**
     * 终止运行时
     *
     * @return
     *
     * @code Python函数原型
     * stop(self: pyaubo_sdk.RuntimeMachine) -> int
     * @endcode
     *
     * @code Lua函数原型
     * stop() -> number
     * @endcode
     */
    int stop();

    /**
     * 终止脚本运行(在关节空间进行规划，以最快速度退出)
     *
     * @return
     *
     * @code Python函数原型
     * abort(self: pyaubo_sdk.RuntimeMachine) -> int
     * @endcode
     *
     * @code Lua函数原型
     * abort() -> number
     * @endcode
     */
    int abort();

    /**
     * 暂停解释器
     *
     * @return
     *
     * @code Python函数原型
     * pause(self: pyaubo_sdk.RuntimeMachine) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pause() -> number
     * @endcode
     */
    int pause();

    /**
     * 单步运行
     *
     * @return
     *
     * @code Python函数原型
     * step(self: pyaubo_sdk.RuntimeMachine) -> int
     * @endcode
     *
     * @code Lua函数原型
     * step() -> number
     * @endcode
     */
    int step();

    /**
     * 恢复解释器
     *
     * @return
     *
     * @code Python函数原型
     * resume(self: pyaubo_sdk.RuntimeMachine) -> int
     * @endcode
     *
     * @code Lua函数原型
     * resume() -> number
     * @endcode
     */
    int resume();

    /**
     * 获取规划器的状态
     *
     * @return
     *
     * @code Python函数原型
     * getStatus(self: pyaubo_sdk.RuntimeMachine) ->
     * arcs::common_interface::RuntimeState
     * @endcode
     *
     * @code Lua函数原型
     * getStatus() -> number
     * @endcode
     */
    RuntimeState getStatus();

    /**
     * 设置断点
     *
     * @param lineno
     * @return
     *
     * @code Python函数原型
     * setBreakPoint(self: pyaubo_sdk.RuntimeMachine, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setBreakPoint(lineno: number) -> number
     * @endcode
     */
    int setBreakPoint(int lineno);

    /**
     * 移除断点
     *
     * @param lineno
     * @return
     *
     * @code Python函数原型
     * removeBreakPoint(self: pyaubo_sdk.RuntimeMachine, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * removeBreakPoint(lineno: number) -> number
     * @endcode
     */
    int removeBreakPoint(int lineno);

    /**
     * 清除所有断点
     *
     * @return
     *
     * @code Python函数原型
     * clearBreakPoints(self: pyaubo_sdk.RuntimeMachine) -> int
     * @endcode
     *
     * @code Lua函数原型
     * clearBreakPoints() -> number
     * @endcode
     */
    int clearBreakPoints();

    /**
     * 定时器开始
     *
     * @param name
     * @return
     *
     * @code Python函数原型
     * timerStart(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * timerStart(name: string) -> nil
     * @endcode
     */
    int timerStart(const std::string &name);

    /**
     * 定时器结束
     *
     * @param name
     * @return
     *
     * @code Python函数原型
     * timerStop(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * timerStop(name: string) -> nil
     * @endcode
     */
    int timerStop(const std::string &name);

    /**
     * 定时器重置
     *
     * @param name
     * @return
     *
     * @code Python函数原型
     * timerReset(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * timerReset(name: string) -> nil
     * @endcode
     */
    int timerReset(const std::string &name);

    /**
     * 定时器删除
     *
     * @param name
     * @return
     *
     * @code Python函数原型
     * timerDelete(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * timerDelete(name: string) -> nil
     * @endcode
     */
    int timerDelete(const std::string &name);

    /**
     * 获取定时器数值
     *
     * @param name
     * @return
     *
     * @code Python函数原型
     * getTimer(self: pyaubo_sdk.RuntimeMachine, arg0: str) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getTimer(name: string) -> number
     * @endcode
     */
    double getTimer(const std::string &name);

protected:
    void *d_;
};

using RuntimeMachinePtr = std::shared_ptr<RuntimeMachine>;

// clang-format off
#define RuntimeMachine_DECLARES                                     \
    _INST(RuntimeMachine, 3, setPlanContext, tid, lineno, comment)  \
    _FUNC(RuntimeMachine, 1, gotoLine, lineno)                      \
    _FUNC(RuntimeMachine, 1, setInstructionAllowed, allowed)        \
    _FUNC(RuntimeMachine, 0, getAdvancePlanContext)                 \
    _FUNC(RuntimeMachine, 0, getAdvancePtr)                         \
    _FUNC(RuntimeMachine, 0, getMainPtr)                            \
    _FUNC(RuntimeMachine, 0, getPlanContext)                        \
    _FUNC(RuntimeMachine, 0, getExecutionStatus)                    \
    _FUNC(RuntimeMachine, 1, loadProgram, program)                  \
    _FUNC(RuntimeMachine, 0, runProgram)                            \
    _FUNC(RuntimeMachine, 0, start)                                 \
    _FUNC(RuntimeMachine, 0, stop)                                  \
    _FUNC(RuntimeMachine, 0, abort)                                 \
    _FUNC(RuntimeMachine, 0, pause)                                 \
    _FUNC(RuntimeMachine, 0, step)                                  \
    _FUNC(RuntimeMachine, 0, resume)                                \
    _FUNC(RuntimeMachine, 0, getStatus)                             \
    _FUNC(RuntimeMachine, 1, setBreakPoint, lineno)                 \
    _FUNC(RuntimeMachine, 1, removeBreakPoint, lineno)              \
    _FUNC(RuntimeMachine, 0, clearBreakPoints)                      \
    _INST(RuntimeMachine, 1, timerStart, name)                      \
    _INST(RuntimeMachine, 1, timerStop, name)                       \
    _INST(RuntimeMachine, 1, timerReset, name)                      \
    _INST(RuntimeMachine, 1, timerDelete, name)                     \
    _FUNC(RuntimeMachine, 1, getTimer, name)
// clang-format on
} // namespace common_interface
} // namespace arcs
#endif // AUBO_SDK_RUNTIME_MACHINE_H
