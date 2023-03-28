#ifndef AUBO_SDK_SYSTEM_INFO_INTERFACE_H
#define AUBO_SDK_SYSTEM_INFO_INTERFACE_H

#include <stdint.h>
#include <string>
#include <memory>

#include <aubo/global_config.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT SystemInfo
{
public:
    SystemInfo();
    virtual ~SystemInfo();

    /**
     * 获取控制器软件版本号
     *
     * @return 返回控制器软件版本号
     *
     * @code C++示例
     * int control_version =
     * rpc_cli->getSystemInfo()->getControlSoftwareVersionCode();
     * @endcode
     *
     * @code Python函数原型
     * getControlSoftwareVersionCode(self: pyaubo_sdk.SystemInfo) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getControlSoftwareVersionCode() -> number
     * @endcode
     */
    int getControlSoftwareVersionCode();

    /**
     * 获取接口版本号
     *
     * @return 返回接口版本号
     *
     * @code C++示例
     * int interface_version =
     * rpc_cli->getSystemInfo()->getInterfaceVersionCode();
     * @endcode
     *
     * @code Python函数原型
     * getInterfaceVersionCode(self: pyaubo_sdk.SystemInfo) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getInterfaceVersionCode() -> number
     * @endcode
     */
    int getInterfaceVersionCode();

    /**
     * 获取控制器软件构建时间
     *
     * @return 返回控制器软件构建时间
     *
     * @code C++示例
     * std::string build_date =
     * rpc_cli->getSystemInfo()->getControlSoftwareBuildDate();
     * @endcode
     *
     * @code Python函数原型
     * getControlSoftwareBuildDate(self: pyaubo_sdk.SystemInfo) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getControlSoftwareBuildDate() -> string
     * @endcode
     */
    std::string getControlSoftwareBuildDate();

    /**
     * 获取控制器软件git版
     *
     * @return 返回控制器软件git版本
     *
     * @code C++示例
     * std::string git_version =
     * rpc_cli->getSystemInfo()->getControlSoftwareVersionHash();
     * @endcode
     *
     * @code Python函数原型
     * getControlSoftwareVersionHash(self: pyaubo_sdk.SystemInfo) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getControlSoftwareVersionHash() -> string
     * @endcode
     */
    std::string getControlSoftwareVersionHash();

    /**
     * 获取系统时间(软件启动时间us)
     *
     * @return 返回系统时间(软件启动时间us)
     *
     * @code C++示例
     * std::string system_time =
     * rpc_cli->getSystemInfo()->getControlSystemTime();
     * @endcode
     *
     * @code Python函数原型
     * getControlSystemTime(self: pyaubo_sdk.SystemInfo) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getControlSystemTime() -> number
     * @endcode
     */
    uint64_t getControlSystemTime();

protected:
    void *d_;
};

using SystemInfoPtr = std::shared_ptr<SystemInfo>;

// clang-format off
#define SystemInfo_DECLARES                              \
    _FUNC(SystemInfo, 0, getControlSoftwareVersionCode)  \
    _FUNC(SystemInfo, 0, getInterfaceVersionCode)        \
    _FUNC(SystemInfo, 0, getControlSoftwareBuildDate)    \
    _FUNC(SystemInfo, 0, getControlSoftwareVersionHash)  \
    _FUNC(SystemInfo, 0, getControlSystemTime)
// clang-format on
} // namespace common_interface
} // namespace arcs
#endif
