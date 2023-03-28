#ifndef AUBO_SDK_IO_CONTROL_INTERFACE_H
#define AUBO_SDK_IO_CONTROL_INTERFACE_H

#include <vector>
#include <thread>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

/**
 * IoControl
 * 提供了一系列的接口对机器人标配的一些数字、模拟IO进行配置，输出状态设置、读取
 *
 * 1. 获取各种IO的数量
 * 2. 配置IO的输入输出功能
 * 3. 可配置IO的配置
 * 4. 模拟IO的输入输出范围设置、读取
 *
 * 标准数字输入输出：控制柜IO面板上的标准IO
 * 工具端数字输入输出：通过工具末端航插暴露的数字IO
 * 可配置输入输出：可以配置为安全IO或者普通数字IO
 */
class ARCS_ABI_EXPORT IoControl
{
public:
    IoControl();
    virtual ~IoControl();

    /**
     * 获取标准数字输入数量
     *
     * @return 返回标准数字输入数量
     *
     * @code Python函数原型
     * getStandardDigitalInputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalInputNum() -> number
     * @endcode
     */
    int getStandardDigitalInputNum();

    /**
     * 获取工具端数字输入数量
     *
     * @return 返回工具端数字输入数量
     *
     * @code Python函数原型
     * getToolDigitalInputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalInputNum() -> number
     * @endcode
     */
    int getToolDigitalInputNum();

    /**
     * 获取可配置数字输入数量
     *
     * @return 返回可配置数字输入数量
     *
     * @code Python函数原型
     * getConfigurableDigitalInputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalInputNum() -> number
     * @endcode
     */
    int getConfigurableDigitalInputNum();

    /**
     * 获取标准数字输出数量
     *
     * @return 返回标准数字输出数量
     *
     * @code Python函数原型
     * getStandardDigitalOutputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalOutputNum() -> number
     * @endcode
     */
    int getStandardDigitalOutputNum();

    /**
     * 获取工具端数字输出数量
     *
     * @return 返回工具端数字输出数量
     *
     * @code Python函数原型
     * getToolDigitalOutputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalOutputNum() -> number
     * @endcode
     */
    int getToolDigitalOutputNum();

    /**
     * 设置指定的工具端IO为输入/输出
     * 工具端IO比较特殊，IO可以配置为输入或者输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param input: 表示指定IO是否为输入
     * input 为true时，设置指定IO为输入，否则为输出
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolIoInput(self: pyaubo_sdk.IoControl, arg0: int, arg1: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolIoInput(index: number, input: boolean) -> nil
     * @endcode
     */
    int setToolIoInput(int index, bool input);

    /**
     * 判断指定IO是否为输入
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 当指定的IO为输入时返回 true, 否则为 false
     *
     * @code Python函数原型
     * isToolIoInput(self: pyaubo_sdk.IoControl, arg0: int) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * isToolIoInput(index: number) -> boolean
     * @endcode
     */
    bool isToolIoInput(int index);

    /**
     * 获取可配置数字输出数量
     *
     * @return 返回可配置数字输出数量
     *
     * @code Python函数原型
     * getConfigurableDigitalOutputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalOutputNum() -> number
     * @endcode
     */
    int getConfigurableDigitalOutputNum();

    /**
     * 获取标准模拟输入数量
     *
     * @return 返回标准模拟输入数量
     *
     * @code Python函数原型
     * getStandardAnalogInputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardAnalogInputNum() -> number
     * @endcode
     */
    int getStandardAnalogInputNum();

    /**
     * 获取工具端模拟输入数量
     *
     * @return 返回工具端模拟输入数量
     *
     * @code Python函数原型
     * getToolAnalogInputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolAnalogInputNum() -> number
     * @endcode
     */
    int getToolAnalogInputNum();

    /**
     * 获取标准模拟输出数量
     *
     * @return 返回标准模拟输出数量
     *
     * @code Python函数原型
     * getStandardAnalogOutputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardAnalogOutputNum() -> number
     * @endcode
     */
    int getStandardAnalogOutputNum();

    /**
     * 获取工具端模拟输出数量
     *
     * @return 返回工具端模拟输出数量
     *
     * @code Python函数原型
     * getToolAnalogOutputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolAnalogOutputNum() -> number
     * @endcode
     */
    int getToolAnalogOutputNum();

    /**
     * 设置所有数字输入动作为默认值
     *
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setDigitalInputActionDefault(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setDigitalInputActionDefault() -> nil
     * @endcode
     */
    int setDigitalInputActionDefault();

    /**
     * 设置标准数字输入触发动作
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param action: 触发动作
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardDigitalInputAction(self: pyaubo_sdk.IoControl, arg0: int,
     * arg1: arcs::common_interface::StandardInputAction) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardDigitalInputAction(index: number, action: number) -> nil
     * @endcode
     */
    int setStandardDigitalInputAction(int index, StandardInputAction action);

    /**
     * 设置工具数字输入触发动作
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param action: 触发动作
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolDigitalInputAction(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * arcs::common_interface::StandardInputAction) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolDigitalInputAction(index: number, action: number) -> nil
     * @endcode
     */
    int setToolDigitalInputAction(int index, StandardInputAction action);

    /**
     * 设置可配置数字输入触发动作
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param action: 触发动作
     * @return 成功返回0；失败返回错误码
     *
     * @note 需要将可配置输入的安全输入动作设置为
     * SafetyInputAction::Unassigned时这个函数的配置才会生效
     *
     * @code Python函数原型
     * setConfigurableDigitalInputAction(self: pyaubo_sdk.IoControl, arg0: int,
     * arg1: arcs::common_interface::StandardInputAction) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setConfigurableDigitalInputAction(index: number, action: number) -> nil
     * @endcode
     */
    int setConfigurableDigitalInputAction(int index,
                                          StandardInputAction action);

    /**
     * 获取标准数字输入的输入触发动作
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回标准数字输入的输入触发动作
     *
     * @code Python函数原型
     * getStandardDigitalInputAction(self: pyaubo_sdk.IoControl, arg0: int) ->
     * arcs::common_interface::StandardInputAction
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalInputAction(index: number) -> number
     * @endcode
     */
    StandardInputAction getStandardDigitalInputAction(int index);

    /**
     * 获取工具端数字输入的输入触发动作
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回工具端数字输入的输入触发动作
     *
     * @code Python函数原型
     * getToolDigitalInputAction(self: pyaubo_sdk.IoControl, arg0: int) ->
     * arcs::common_interface::StandardInputAction
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalInputAction(index: number) -> number
     * @endcode
     */
    StandardInputAction getToolDigitalInputAction(int index);

    /**
     * 获取可配置数字输入的输入触发动作
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回输入触发动作
     *
     * @code Python函数原型
     * getConfigurableDigitalInputAction(self: pyaubo_sdk.IoControl, arg0: int)
     * -> arcs::common_interface::StandardInputAction
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalInputAction(index: number) -> number
     * @endcode
     */
    StandardInputAction getConfigurableDigitalInputAction(int index);

    /**
     * 设置所有数字输出动作为默认值
     *
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setDigitalOutputRunstateDefault(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setDigitalOutputRunstateDefault() -> nil
     * @endcode
     */
    int setDigitalOutputRunstateDefault();

    /**
     * 设置标准数字输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param runstate: 输出状态选择
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardDigitalOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int,
     * arg1: arcs::common_interface::StandardOutputRunState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardDigitalOutputRunstate(index: number, runstate: number) -> nil
     * @endcode
     */
    int setStandardDigitalOutputRunstate(int index,
                                         StandardOutputRunState runstate);

    /**
     * 设置工具端数字输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param runstate: 输出状态选择
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolDigitalOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * arcs::common_interface::StandardOutputRunState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolDigitalOutputRunstate(index: number, runstate: number) -> nil
     * @endcode
     */
    int setToolDigitalOutputRunstate(int index,
                                     StandardOutputRunState runstate);

    /**
     * 设置可配置数字输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param runstate: 输出状态选择
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setConfigurableDigitalOutputRunstate(self: pyaubo_sdk.IoControl, arg0:
     * int, arg1: arcs::common_interface::StandardOutputRunState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setConfigurableDigitalOutputRunstate(index: number, runstate: number) ->
     * nil
     * @endcode
     */
    int setConfigurableDigitalOutputRunstate(int index,
                                             StandardOutputRunState runstate);

    /**
     * 获取标准数字输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回输出状态选择
     *
     * @code Python函数原型
     * getStandardDigitalOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int)
     * -> arcs::common_interface::StandardOutputRunState
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalOutputRunstate(index: number) -> number
     * @endcode
     */
    StandardOutputRunState getStandardDigitalOutputRunstate(int index);

    /**
     * 获取工具端数字输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回输出状态选择
     *
     * @code Python函数原型
     * getToolDigitalOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int) ->
     * arcs::common_interface::StandardOutputRunState
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalOutputRunstate(index: number) -> number
     * @endcode
     */
    StandardOutputRunState getToolDigitalOutputRunstate(int index);

    /**
     * 获取可配置数字输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回输出状态选择
     *
     * @code Python函数原型
     * getConfigurableDigitalOutputRunstate(self: pyaubo_sdk.IoControl, arg0:
     * int)
     * -> arcs::common_interface::StandardOutputRunState
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalOutputRunstate(index: number) -> number
     * @endcode
     */
    StandardOutputRunState getConfigurableDigitalOutputRunstate(int index);

    /**
     * 设置标准模拟输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param runstate: 输出状态选择
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardAnalogOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int,
     * arg1: arcs::common_interface::StandardOutputRunState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardAnalogOutputRunstate(index: number, runstate: number) -> nil
     * @endcode
     */
    int setStandardAnalogOutputRunstate(int index,
                                        StandardOutputRunState runstate);

    /**
     * 设置工具端模拟输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param runstate: 输出状态选择
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolAnalogOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * arcs::common_interface::StandardOutputRunState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolAnalogOutputRunstate(index: number, runstate: number) -> nil
     * @endcode
     */
    int setToolAnalogOutputRunstate(int index, StandardOutputRunState runstate);

    /**
     * 获取标准模拟输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回标准模拟输出状态选择
     *
     * @code Python函数原型
     * getStandardAnalogOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int) ->
     * arcs::common_interface::StandardOutputRunState
     * @endcode
     *
     * @code Lua函数原型
     * getStandardAnalogOutputRunstate(index: number) -> number
     * @endcode
     */
    StandardOutputRunState getStandardAnalogOutputRunstate(int index);

    /**
     * 获取工具端模拟输出状态选择
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回工具端模拟输出状态选择
     *
     * @code Python函数原型
     * getToolAnalogOutputRunstate(self: pyaubo_sdk.IoControl, arg0: int) ->
     * arcs::common_interface::StandardOutputRunState
     * @endcode
     *
     * @code Lua函数原型
     * getToolAnalogOutputRunstate(index: number) -> number
     * @endcode
     */
    StandardOutputRunState getToolAnalogOutputRunstate(int index);

    /**
     * 设置标准模拟输入的范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param domain: 输入的范围
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardAnalogInputDomain(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardAnalogInputDomain(index: number, domain: number) -> nil
     * @endcode
     */
    int setStandardAnalogInputDomain(int index, int domain);

    /**
     * 设置工具端模拟输入的范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param domain: 输入的范围
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolAnalogInputDomain(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolAnalogInputDomain(index: number, domain: number) -> nil
     * @endcode
     */
    int setToolAnalogInputDomain(int index, int domain);

    /**
     * 获取标准模式输入范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回标准模式输入范围
     *
     * @code Python函数原型
     * getStandardAnalogInputDomain(self: pyaubo_sdk.IoControl, arg0: int) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardAnalogInputDomain(index: number) -> number
     * @endcode
     */
    int getStandardAnalogInputDomain(int index);

    /**
     * 获取工具端模式输入范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回工具端模式输入范围
     *
     * @code Python函数原型
     * getToolAnalogInputDomain(self: pyaubo_sdk.IoControl, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolAnalogInputDomain(index: number) -> number
     * @endcode
     */
    int getToolAnalogInputDomain(int index);

    /**
     * 设置标准模拟输出的范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param domain: 输出的范围
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardAnalogOutputDomain(self: pyaubo_sdk.IoControl, arg0: int,
     * arg1: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardAnalogOutputDomain(index: number, domain: number) -> nil
     * @endcode
     */
    int setStandardAnalogOutputDomain(int index, int domain);

    /**
     * 设置工具端模拟输出范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param domain: 输出的范围
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolAnalogOutputDomain(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolAnalogOutputDomain(index: number, domain: number) -> nil
     * @endcode
     */
    int setToolAnalogOutputDomain(int index, int domain);

    /**
     * 获取标准模拟输出范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回标准模拟输出范围
     *
     * @code Python函数原型
     * getStandardAnalogOutputDomain(self: pyaubo_sdk.IoControl, arg0: int) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardAnalogOutputDomain(index: number) -> number
     * @endcode
     */
    int getStandardAnalogOutputDomain(int index);

    /**
     * 获取工具端模拟输出范围
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回工具端模拟输出范围
     *
     * @code Python函数原型
     * getToolAnalogOutputDomain(self: pyaubo_sdk.IoControl, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolAnalogOutputDomain(index: number) -> number
     * @endcode
     */
    int getToolAnalogOutputDomain(int index);

    /**
     * 设置工具端电源电压
     *
     * @param domain: 可选三个档位 0: 0V, 12: 12V, 24: 24V
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setToolVoltageOutputDomain(self: pyaubo_sdk.IoControl, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolVoltageOutputDomain(domain: number) -> nil
     * @endcode
     */
    int setToolVoltageOutputDomain(int domain);

    /**
     * 获取工具端电源电压
     *
     * @return 返回工具端电源电压
     *
     * @code Python函数原型
     * getToolVoltageOutputDomain(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolVoltageOutputDomain() -> number
     * @endcode
     */
    int getToolVoltageOutputDomain();

    /**
     * 设置标准数字输出
     *
     * @param index:  表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param value: 输出
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardDigitalOutput(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardDigitalOutput(index: number, value: boolean) -> nil
     * @endcode
     */
    int setStandardDigitalOutput(int index, bool value);

    /**
     * 设置数字输出脉冲
     *
     * @param index
     * @param value
     * @param duration
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardDigitalOutputPulse(self: pyaubo_sdk.IoControl, arg0: int,
     * arg1: bool, arg2: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardDigitalOutputPulse(index: number, value: boolean, duration:
     * number) -> nil
     * @endcode
     */
    int setStandardDigitalOutputPulse(int index, bool value, double duration);

    /**
     * 设置工具端数字输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param value: 数字输出
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolDigitalOutput(self: pyaubo_sdk.IoControl, arg0: int, arg1: bool)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolDigitalOutput(index: number, value: boolean) -> nil
     * @endcode
     */
    int setToolDigitalOutput(int index, bool value);

    /**
     * 设置工具端数字输出脉冲
     *
     * @param index
     * @param value
     * @param duration
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolDigitalOutputPulse(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * bool, arg2: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolDigitalOutputPulse(index: number, value: boolean, duration:
     * number) -> nil
     * @endcode
     */
    int setToolDigitalOutputPulse(int index, bool value, double duration);

    /**
     * 设置可配置数字输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param value: 数字输出
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setConfigurableDigitalOutput(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setConfigurableDigitalOutput(index: number, value: boolean) -> nil
     * @endcode
     */
    int setConfigurableDigitalOutput(int index, bool value);

    /**
     * 设置可配置数字输出脉冲
     *
     * @param index
     * @param value
     * @param duration
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setConfigurableDigitalOutputPulse(self: pyaubo_sdk.IoControl, arg0: int,
     * arg1: bool, arg2: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setConfigurableDigitalOutputPulse(index: number, value: boolean,
     * duration: number) -> nil
     * @endcode
     */
    int setConfigurableDigitalOutputPulse(int index, bool value,
                                          double duration);

    /**
     * 设置标准模拟输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param value: 模拟输出
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setStandardAnalogOutput(self: pyaubo_sdk.IoControl, arg0: int, arg1:
     * float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setStandardAnalogOutput(index: number, value: number) -> nil
     * @endcode
     */
    int setStandardAnalogOutput(int index, double value);

    /**
     * 设置工具端模拟输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @param value: 模拟输出
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setToolAnalogOutput(self: pyaubo_sdk.IoControl, arg0: int, arg1: float)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolAnalogOutput(index: number, value: number) -> nil
     * @endcode
     */
    int setToolAnalogOutput(int index, double value);

    /**
     * 获取标准数字输入
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 高电平返回true; 低电平返回false
     *
     * @code Python函数原型
     * getStandardDigitalInput(self: pyaubo_sdk.IoControl, arg0: int) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalInput(index: number) -> boolean
     * @endcode
     */
    bool getStandardDigitalInput(int index);

    /**
     * 获取所有的标准数字输入
     *
     * @return 返回所有的标准数字输入
     * 例如，当返回值是2863267846时,换成2进制后是1010101010101010
     * 0000000000000110. 后16位就是所有的标准数字输入状态值.
     * 最后一位表示DI00的输入状态值,倒数第二位表示DI01的输入状态值，以此类推.
     * 1表示高电平状态，0表示低电平状态.
     *
     * @code Python函数原型
     * getStandardDigitalInputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalInputs() -> number
     * @endcode
     */
    uint32_t getStandardDigitalInputs();

    /**
     * 获取工具端数字输入
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 高电平返回true; 低电平返回false
     *
     * @code Python函数原型
     * getToolDigitalInput(self: pyaubo_sdk.IoControl, arg0: int) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalInput(index: number) -> boolean
     * @endcode
     */
    bool getToolDigitalInput(int index);

    /**
     *  获取所有的工具端数字输入
     *
     * @return 返回所有的工具端数字输入
     *
     * @code Python函数原型
     * getToolDigitalInputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalInputs() -> number
     * @endcode
     */
    uint32_t getToolDigitalInputs();

    /**
     * 获取可配置数字输入
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 高电平返回true; 低电平返回false
     *
     * @code Python函数原型
     * getConfigurableDigitalInput(self: pyaubo_sdk.IoControl, arg0: int) ->
     * bool
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalInput(index: number) -> boolean
     * @endcode
     */
    bool getConfigurableDigitalInput(int index);

    /**
     * 获取所有的可配置数字输入
     *
     * @return 返回所有的可配置数字输入
     *
     * @code Python函数原型
     * getConfigurableDigitalInputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalInputs() -> number
     * @endcode
     */
    uint32_t getConfigurableDigitalInputs();

    /**
     * 获取标准数字输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 高电平返回true; 低电平返回false
     *
     * @code Python函数原型
     * getStandardDigitalOutput(self: pyaubo_sdk.IoControl, arg0: int) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalOutput(index: number) -> boolean
     * @endcode
     */
    bool getStandardDigitalOutput(int index);

    /**
     * 获取所有的标准数字输出
     *
     * @return 返回所有的标准数字输出
     * 例如，当返回值是2863267846时,换成2进制后是1010101010101010
     * 0000000000000110. 后16位就是所有的标准数字输出状态值.
     * 最后一位表示DI00的输出状态值,倒数第二位表示DI01的输出状态值，以此类推.
     * 1表示高电平状态，0表示低电平状态.
     *
     * @code Python函数原型
     * getStandardDigitalOutputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStandardDigitalOutputs() -> number
     * @endcode
     */
    uint32_t getStandardDigitalOutputs();

    /**
     * 获取工具端数字输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 高电平返回true; 低电平返回false
     *
     * @code Python函数原型
     * getToolDigitalOutput(self: pyaubo_sdk.IoControl, arg0: int) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalOutput(index: number) -> boolean
     * @endcode
     */
    bool getToolDigitalOutput(int index);

    /**
     * 获取所有的工具端数字输出
     *
     * @return 返回所有的工具端数字输出
     *
     * @code Python函数原型
     * getToolDigitalOutputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolDigitalOutputs() -> number
     * @endcode
     */
    uint32_t getToolDigitalOutputs();

    /**
     * 获取可配值数字输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 高电平返回true; 低电平返回false
     *
     * @code Python函数原型
     * getConfigurableDigitalOutput(self: pyaubo_sdk.IoControl, arg0: int) ->
     * bool
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalOutput(index: number) -> boolean
     * @endcode
     */
    bool getConfigurableDigitalOutput(int index);

    /**
     * 获取所有的可配值数字输出
     *
     * @return 返回所有的可配值数字输出
     *
     * @code Python函数原型
     * getConfigurableDigitalOutputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getConfigurableDigitalOutputs() -> number
     * @endcode
     */
    uint32_t getConfigurableDigitalOutputs();

    /**
     * 获取标准模拟输入
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回标准模拟输入值
     *
     * @code Python函数原型
     * getStandardAnalogInput(self: pyaubo_sdk.IoControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getStandardAnalogInput(index: number) -> number
     * @endcode
     */
    double getStandardAnalogInput(int index);

    /**
     * 获取工具端模拟输入
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回工具端模拟输入
     *
     * @code Python函数原型
     * getToolAnalogInput(self: pyaubo_sdk.IoControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getToolAnalogInput(index: number) -> number
     * @endcode
     */
    double getToolAnalogInput(int index);

    /**
     * 获取标准模拟输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回标准模拟输出
     *
     * @code Python函数原型
     * getStandardAnalogOutput(self: pyaubo_sdk.IoControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getStandardAnalogOutput(index: number) -> number
     * @endcode
     */
    double getStandardAnalogOutput(int index);

    /**
     * 获取工具端模拟输出
     *
     * @param index: 表示IO口的管脚，可用十进制或者十六进制表示
     * 例如，0x00000000 表示第一个管脚
     * @return 返回工具端模拟输出
     *
     * @code Python函数原型
     * getToolAnalogOutput(self: pyaubo_sdk.IoControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getToolAnalogOutput(index: number) -> number
     * @endcode
     */
    double getToolAnalogOutput(int index);

    /**
     * 获取联动输入数量
     *
     * @return 返回联动输入数量
     *
     * @note 兼容接口板，仅供读数显示
     *
     * @code Python函数原型
     * getStaticLinkInputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStaticLinkInputNum() -> number
     * @endcode
     */
    int getStaticLinkInputNum();

    /**
     * 获取联动输出数量
     *
     * @return 返回联动输出数量
     *
     * @note 兼容接口板，仅供读数显示
     *
     * @code Python函数原型
     * getStaticLinkOutputNum(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStaticLinkOutputNum() -> number
     * @endcode
     */
    int getStaticLinkOutputNum();

    /**
     * 获取所有的联动输入
     *
     * @return 返回所有的联动输入
     *
     * @note 兼容接口板，仅供读数显示
     *
     * @code Python函数原型
     * getStaticLinkInputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStaticLinkInputs() -> number
     * @endcode
     */
    uint32_t getStaticLinkInputs();

    /**
     * 获取所有的联动输出
     *
     * @return 返回所有的联动输出
     *
     * @note 兼容接口板，仅供读数显示
     *
     * @code Python函数原型
     * getStaticLinkOutputs(self: pyaubo_sdk.IoControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getStaticLinkOutputs() -> number
     * @endcode
     */
    uint32_t getStaticLinkOutputs();

protected:
    void *d_;
};
using IoControlPtr = std::shared_ptr<IoControl>;

// clang-format off
#define IoControl_DECLARES                                                   \
    _FUNC(IoControl, 0, getStandardDigitalInputNum)                          \
    _FUNC(IoControl, 0, getToolDigitalInputNum)                              \
    _FUNC(IoControl, 0, getConfigurableDigitalInputNum)                      \
    _FUNC(IoControl, 0, getStandardDigitalOutputNum)                         \
    _FUNC(IoControl, 0, getToolDigitalOutputNum)                             \
    _INST(IoControl, 2, setToolIoInput, index, input)                        \
    _FUNC(IoControl, 1, isToolIoInput, index)                                \
    _FUNC(IoControl, 0, getConfigurableDigitalOutputNum)                     \
    _FUNC(IoControl, 0, getStandardAnalogInputNum)                           \
    _FUNC(IoControl, 0, getToolAnalogInputNum)                               \
    _FUNC(IoControl, 0, getStandardAnalogOutputNum)                          \
    _FUNC(IoControl, 0, getToolAnalogOutputNum)                              \
    _INST(IoControl, 0, setDigitalInputActionDefault)                        \
    _INST(IoControl, 2, setStandardDigitalInputAction, index, action)        \
    _INST(IoControl, 2, setToolDigitalInputAction, index, action)            \
    _INST(IoControl, 2, setConfigurableDigitalInputAction, index, action)    \
    _FUNC(IoControl, 1, getStandardDigitalInputAction, index)                \
    _FUNC(IoControl, 1, getToolDigitalInputAction, index)                    \
    _FUNC(IoControl, 1, getConfigurableDigitalInputAction, index)            \
    _INST(IoControl, 0, setDigitalOutputRunstateDefault)                     \
    _INST(IoControl, 2, setStandardDigitalOutputRunstate, index, runstate)   \
    _INST(IoControl, 2, setToolDigitalOutputRunstate, index, runstate)       \
    _INST(IoControl, 2, setConfigurableDigitalOutputRunstate, index, runstate) \
    _FUNC(IoControl, 1, getStandardDigitalOutputRunstate, index)             \
    _FUNC(IoControl, 1, getToolDigitalOutputRunstate, index)                 \
    _FUNC(IoControl, 1, getConfigurableDigitalOutputRunstate, index)         \
    _INST(IoControl, 2, setStandardAnalogOutputRunstate, index, runstate)    \
    _INST(IoControl, 2, setToolAnalogOutputRunstate, index, runstate)        \
    _FUNC(IoControl, 1, getStandardAnalogOutputRunstate, index)              \
    _FUNC(IoControl, 1, getToolAnalogOutputRunstate, index)                  \
    _INST(IoControl, 2, setStandardAnalogInputDomain, index, domain)         \
    _INST(IoControl, 2, setToolAnalogInputDomain, index, domain)             \
    _FUNC(IoControl, 1, getStandardAnalogInputDomain, index)                 \
    _FUNC(IoControl, 1, getToolAnalogInputDomain, index)                     \
    _INST(IoControl, 2, setStandardAnalogOutputDomain, index, domain)        \
    _INST(IoControl, 2, setToolAnalogOutputDomain, index, domain)            \
    _INST(IoControl, 1, setToolVoltageOutputDomain, domain)                  \
    _FUNC(IoControl, 0, getToolVoltageOutputDomain)                          \
    _FUNC(IoControl, 1, getStandardAnalogOutputDomain, index)                \
    _FUNC(IoControl, 1, getToolAnalogOutputDomain, index)                    \
    _INST(IoControl, 2, setStandardDigitalOutput, index, value)              \
    _INST(IoControl, 3, setStandardDigitalOutputPulse, index, value, duration) \
    _INST(IoControl, 2, setToolDigitalOutput, index, value)                  \
    _INST(IoControl, 3, setToolDigitalOutputPulse, index, value, duration)   \
    _INST(IoControl, 2, setConfigurableDigitalOutput, index, value)          \
    _INST(IoControl, 3, setConfigurableDigitalOutputPulse, index, value, duration) \
    _INST(IoControl, 2, setStandardAnalogOutput, index, value)               \
    _INST(IoControl, 2, setToolAnalogOutput, index, value)                   \
    _FUNC(IoControl, 1, getStandardDigitalInput, index)                      \
    _FUNC(IoControl, 0, getStandardDigitalInputs)                            \
    _FUNC(IoControl, 1, getToolDigitalInput, index)                          \
    _FUNC(IoControl, 0, getToolDigitalInputs)                                \
    _FUNC(IoControl, 1, getConfigurableDigitalInput, index)                  \
    _FUNC(IoControl, 0, getConfigurableDigitalInputs)                        \
    _FUNC(IoControl, 1, getStandardAnalogInput, index)                       \
    _FUNC(IoControl, 1, getToolAnalogInput, index)                           \
    _FUNC(IoControl, 1, getStandardDigitalOutput, index)                     \
    _FUNC(IoControl, 0, getStandardDigitalOutputs)                           \
    _FUNC(IoControl, 1, getToolDigitalOutput, index)                         \
    _FUNC(IoControl, 0, getToolDigitalOutputs)                               \
    _FUNC(IoControl, 1, getConfigurableDigitalOutput, index)                 \
    _FUNC(IoControl, 0, getConfigurableDigitalOutputs)                       \
    _FUNC(IoControl, 1, getStandardAnalogOutput, index)                      \
    _FUNC(IoControl, 1, getToolAnalogOutput, index)                          \
    _FUNC(IoControl, 0, getStaticLinkInputNum)                               \
    _FUNC(IoControl, 0, getStaticLinkOutputNum)                              \
    _FUNC(IoControl, 0, getStaticLinkInputs)                                 \
    _FUNC(IoControl, 0, getStaticLinkOutputs)
// clang-format on
} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_IO_CONTROL_INTERFACE_H
