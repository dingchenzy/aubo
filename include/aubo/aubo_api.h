#ifndef AUBO_SDK_AUBO_API_INTERFACE_H
#define AUBO_SDK_AUBO_API_INTERFACE_H

#include <aubo/system_info.h>
#include <aubo/runtime_machine.h>
#include <aubo/register_control.h>
#include <aubo/robot_interface.h>
#include <aubo/global_config.h>
#include <aubo/math.h>
#include <aubo/socket.h>
#include <aubo/serial.h>

namespace arcs {
namespace common_interface {

/**
 * ARCS 机器人及外部轴等控制API接口
 */
class ARCS_ABI_EXPORT AuboApi
{
public:
    AuboApi();
    virtual ~AuboApi();

    /**
     * 获取纯数学相关接口
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * MathPtr ptr = rpc_cli->getMath();
     * @endcode
     *
     * @code Python函数原型
     * getMath(self: pyaubo_sdk.AuboApi) -> pyaubo_sdk.Math
     * @endcode
     */
    MathPtr getMath();

    /**
     * 获取系统信息
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * SystemInfoPtr ptr = rpc_cli->getSystemInfo();
     * @endcode
     *
     * @code Python函数原型
     * getSystemInfo(self: pyaubo_sdk.AuboApi) -> pyaubo_sdk.SystemInfo
     * @endcode
     */
    SystemInfoPtr getSystemInfo();

    /**
     * 获取运行时接口
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * RuntimeMachinePtr ptr = rpc_cli->getRuntimeMachine();
     * @endcode
     *
     * @code Python函数原型
     * getRuntimeMachine(self: pyaubo_sdk.AuboApi) -> pyaubo_sdk.RuntimeMachine
     * @endcode
     */
    RuntimeMachinePtr getRuntimeMachine();

    /**
     * 对外寄存器接口
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * RegisterControlPtr ptr = rpc_cli->getRegisterControl();
     * @endcode
     *
     * @code Python函数原型
     * getRegisterControl(self: pyaubo_sdk.AuboApi) ->
     * pyaubo_sdk.RegisterControl
     * @endcode
     */
    RegisterControlPtr getRegisterControl();

    /**
     * 获取机器人列表
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * @endcode
     *
     * @code Python函数原型
     * getRobotNames(self: pyaubo_sdk.AuboApi) -> List[str]
     * @endcode
     */
    std::vector<std::string> getRobotNames();

    /**
     * 根据名字获取 RobotInterfacePtr 接口
     *
     * @param name 机器人名字
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * RobotInterfacePtr ptr = rpc_cli->getRobotInterface(robot_name);
     * @endcode
     *
     * @code Python函数原型
     * getRobotInterface(self: pyaubo_sdk.AuboApi, arg0: str) ->
     * pyaubo_sdk.RobotInterface
     * @endcode
     */
    RobotInterfacePtr getRobotInterface(const std::string &name);

    /// 获取外部轴接口

    /// 获取独立 IO 模块接口

    /**
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * SocketPtr ptr = rpc_cli->getSocket();
     * @endcode
     *
     * @code Python函数原型
     * getSocket(self: pyaubo_sdk.AuboApi) -> arcs::common_interface::Socket
     * @endcode
     */
    SocketPtr getSocket();

    /**
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * SerialPtr ptr = rpc_cli->getSerial();
     * @endcode
     *
     * @code Python函数原型
     * getSerial(self: pyaubo_sdk.AuboApi) -> arcs::common_interface::Serial
     * @endcode
     */
    SerialPtr getSerial();

protected:
    void *d_{ nullptr };
};
using AuboApiPtr = std::shared_ptr<AuboApi>;

// clang-format off
#define AuboApi_DECLARES                       \
    _FUNC(AuboApi, 0, getMath)                 \
    _FUNC(AuboApi, 0, getSystemInfo)           \
    _FUNC(AuboApi, 0, getRuntimeMachine)       \
    _FUNC(AuboApi, 0, getRegisterControl)      \
    _FUNC(AuboApi, 0, getRobotNames)           \
    _FUNC(AuboApi, 1, getRobotInterface, name) \
    _FUNC(AuboApi, 0, getSocket)               \
    _FUNC(AuboApi, 0, getSerial)
// clang-format on

} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_AUBO_API_H
