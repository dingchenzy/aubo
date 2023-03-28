#ifndef AUBO_SDK_ROBOT_CONTROL_INTERFACE_H
#define AUBO_SDK_ROBOT_CONTROL_INTERFACE_H

#include <vector>
#include <thread>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT RobotManage
{
public:
    RobotManage();
    virtual ~RobotManage();

    /**
     * 发起机器人上电请求
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->poweron();
     * @endcode
     *
     * @code Python函数原型
     * poweron(self: pyaubo_sdk.RobotManage) -> int
     * @endcode
     *
     * @code Lua函数原型
     * poweron()
     * @endcode
     */
    int poweron();

    /**
     * 发起机器人启动请求
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->startup();
     * @endcode
     *
     * @code Python函数原型
     * startup(self: pyaubo_sdk.RobotManage) -> int
     * @endcode
     *
     * @code Lua函数原型
     * startup() -> number
     * @endcode
     */
    int startup();

    /**
     * 发起机器人断电请求
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->poweroff();
     * @endcode
     *
     * @code Python函数原型
     * poweroff(self: pyaubo_sdk.RobotManage) -> int
     * @endcode
     *
     * @code Lua函数原型
     * poweroff() -> number
     * @endcode
     */
    int poweroff();

    /**
     * 发起机器人反向驱动请求
     *
     * @param enable
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->backdrive(true);
     * @endcode
     *
     * @code Python函数原型
     * backdrive(self: pyaubo_sdk.RobotManage, arg0: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * backdrive(enable: boolean) -> number
     * @endcode
     */
    int backdrive(bool enable);

    /**
     * 发起机器人自由驱动请求
     *
     * @param enable
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->freedrive(true);
     * @endcode
     *
     * @code Python函数原型
     * freedrive(self: pyaubo_sdk.RobotManage, arg0: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * freedrive(enable: boolean) -> number
     * @endcode
     */
    int freedrive(bool enable);

    /**
     * 发起机器人进入/退出仿真模式请求
     *
     * @param enable
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->setSim(true);
     * @endcode
     *
     * @code Python函数原型
     * setSim(self: pyaubo_sdk.RobotManage, arg0: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setSim(enable: boolean) -> number
     * @endcode
     */
    int setSim(bool enable);

    /**
     * 设置机器人操作模式
     *
     * @param mode 操作模式
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->setOperationalMode(OperationalModeType::Automatic);
     * @endcode
     *
     * @code Python函数原型
     * setOperationalMode(self: pyaubo_sdk.RobotManage, arg0:
     * arcs::common_interface::OperationalModeType) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setOperationalMode(mode: number) -> number
     * @endcode
     */
    int setOperationalMode(OperationalModeType mode);

    /**
     * 获取机器人操作模式
     *
     * @return 机器人操作模式
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * OperationalModeType mode =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->getOperationalMode();
     * @endcode
     *
     * @code Python函数原型
     * getOperationalMode(self: pyaubo_sdk.RobotManage) ->
     * arcs::common_interface::OperationalModeType
     * @endcode
     *
     * @code Lua函数原型
     * getOperationalMode() -> number
     * @endcode
     */
    OperationalModeType getOperationalMode();

    /**
     * 获取控制模式
     *
     * @return 控制模式
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * RobotControlModeType mode =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->getRobotControlMode();
     * @endcode
     *
     * @code Python函数原型
     * getRobotControlMode(self: pyaubo_sdk.RobotManage) ->
     * arcs::common_interface::RobotControlModeType
     * @endcode
     *
     * @code Lua函数原型
     * getRobotControlMode() -> number
     * @endcode
     */
    RobotControlModeType getRobotControlMode();

    /**
     * 是否使能了拖动示教模式
     *
     * @return 使能返回true; 反之返回false
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->isFreedriveEnabled();
     * @endcode
     *
     * @code Python函数原型
     * isFreedriveEnabled(self: pyaubo_sdk.RobotManage) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * isFreedriveEnabled() -> boolean
     * @endcode
     */
    bool isFreedriveEnabled();

    /**
     * 是否使能了反向驱动模式
     *
     * @return 使能返回true; 反之返回false
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->isBackdriveEnabled();
     * @endcode
     *
     * @code Python函数原型
     * isBackdriveEnabled(self: pyaubo_sdk.RobotManage) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * isBackdriveEnabled() -> boolean
     * @endcode
     */
    bool isBackdriveEnabled();

    /**
     * 是否使能了仿真模式
     *
     * @return 使能返回true; 反之返回false
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * bool isEnabled =
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->isSimulationEnabled();
     * @endcode
     *
     * @code Python函数原型
     * isSimulationEnabled(self: pyaubo_sdk.RobotManage) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * isSimulationEnabled() -> boolean
     * @endcode
     */
    bool isSimulationEnabled();

    /**
     * 清除防护停机，包括碰撞停机
     *
     * @return
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->setUnlockProtectiveStop();
     * @endcode
     *
     * @code Python函数原型
     * setUnlockProtectiveStop(self: pyaubo_sdk.RobotManage) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setUnlockProtectiveStop() -> number
     * @endcode
     */
    int setUnlockProtectiveStop();

    /**
     * 重置安全接口板，一般在机器人断电之后需要重置时调用，比如机器人急停、故障等之后
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code C++示例
     * auto rpc_cli = std::make_shared<RpcClient>();
     * auto robot_name = rpc_cli->getRobotNames().front();
     * rpc_cli->getRobotInterface(robot_name)->getRobotManage()->restartInterfaceBoard();
     * @endcode
     *
     * @code Python函数原型
     * restartInterfaceBoard(self: pyaubo_sdk.RobotManage) -> int
     * @endcode
     *
     * @code Lua函数原型
     * restartInterfaceBoard() -> number
     * @endcode
     */
    int restartInterfaceBoard();

protected:
    void *d_;
};
using RobotManagePtr = std::shared_ptr<RobotManage>;

// clang-format off
#define RobotManage_DECLARES                         \
    _FUNC(RobotManage, 0, poweron)                   \
    _FUNC(RobotManage, 0, startup)                   \
    _FUNC(RobotManage, 0, poweroff)                  \
    _FUNC(RobotManage, 1, backdrive, enable)         \
    _FUNC(RobotManage, 1, freedrive, enable)         \
    _FUNC(RobotManage, 1, setSim, enable)            \
    _FUNC(RobotManage, 1, setOperationalMode, mode)  \
    _FUNC(RobotManage, 0, getOperationalMode)        \
    _FUNC(RobotManage, 0, getRobotControlMode)       \
    _FUNC(RobotManage, 0, isSimulationEnabled)       \
    _FUNC(RobotManage, 0, isFreedriveEnabled)        \
    _FUNC(RobotManage, 0, isBackdriveEnabled)        \
    _FUNC(RobotManage, 0, setUnlockProtectiveStop)   \
    _FUNC(RobotManage, 0, restartInterfaceBoard)
// clang-format on
} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_ROBOT_CONTROL_INTERFACE_H
