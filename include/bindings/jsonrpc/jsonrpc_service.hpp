#ifndef COMMON_INTERFACE_JSONRPC_SERVICE_H
#define COMMON_INTERFACE_JSONRPC_SERVICE_H

#include <iostream>

#include <nlohmann/json.hpp>
#include <aubo/aubo_api.h>
#include <bindings/jsonrpc/jsonrpccxx_ex.h>

namespace arcs {
namespace common_interface {

#define RPC_BIND0(prefix, ptr, T, m)                    \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m, \
                       GetHandle(&T::m, *ptr->get##T()), {});
#define RPC_BIND1(prefix, ptr, T, m, arg1)              \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m, \
                       GetHandle(&T::m, *ptr->get##T()), { #arg1 })
#define RPC_BIND2(prefix, ptr, T, m, arg1, arg2)        \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m, \
                       GetHandle(&T::m, *ptr->get##T()), { #arg1, #arg2 })
#define RPC_BIND3(prefix, ptr, T, m, arg1, arg2, arg3)   \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m,  \
                       GetHandle(&T::m, *ptr->get##T()), \
                       { #arg1, #arg2, #arg3 })
#define RPC_BIND4(prefix, ptr, T, m, arg1, arg2, arg3, arg4) \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m,      \
                       GetHandle(&T::m, *ptr->get##T()),     \
                       { #arg1, #arg2, #arg3, #arg4 })
#define RPC_BIND5(prefix, ptr, T, m, arg1, arg2, arg3, arg4, arg5) \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m,            \
                       GetHandle(&T::m, *ptr->get##T()),           \
                       { #arg1, #arg2, #arg3, #arg4, #arg5 })
#define RPC_BIND6(prefix, ptr, T, m, arg1, arg2, arg3, arg4, arg5, arg6) \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m,                  \
                       GetHandle(&T::m, *ptr->get##T()),                 \
                       { #arg1, #arg2, #arg3, #arg4, #arg5, #arg6 })
#define RPC_BIND10(prefix, ptr, T, m, arg1, arg2, arg3, arg4, arg5, arg6, \
                   arg7, arg8, arg9, arg10)                               \
    JsonRpcServer::Add(std::string(prefix) + #T "." #m,                   \
                       GetHandle(&T::m, *ptr->get##T()),                  \
                       { #arg1, #arg2, #arg3, #arg4, #arg5, #arg6, #arg7, \
                         #arg8, #arg9, #arg10 })

// 生成示例代码
#define RPC_SNIPPETES0(prefix, ptr, T, m) \
    os << prefix << #T "." #m << "()" << std::endl;
#define RPC_SNIPPETES1(prefix, ptr, T, m, arg1) \
    os << prefix << #T "." #m << "(" << #arg1 << ")" << std::endl;
#define RPC_SNIPPETES2(prefix, ptr, T, m, arg1, arg2)                 \
    os << prefix << #T "." #m << "(" << #arg1 << ", " << #arg2 << ")" \
       << std::endl;
#define RPC_SNIPPETES3(prefix, ptr, T, m, arg1, arg2, arg3)            \
    os << prefix << #T "." #m << "(" << #arg1 << ", " << #arg2 << ", " \
       << #arg3 << ")" << std::endl;
#define RPC_SNIPPETES4(prefix, ptr, T, m, arg1, arg2, arg3, arg4)      \
    os << prefix << #T "." #m << "(" << #arg1 << ", " << #arg2 << ", " \
       << #arg3 << ", " << #arg4 << ")" << std::endl;
#define RPC_SNIPPETES5(prefix, ptr, T, m, arg1, arg2, arg3, arg4, arg5) \
    os << prefix << #T "." #m << "(" << #arg1 << ", " << #arg2 << ", "  \
       << #arg3 << ", " << #arg4 << ", " << #arg5 << ")" << std::endl;
#define RPC_SNIPPETES6(prefix, ptr, T, m, arg1, arg2, arg3, arg4, arg5, arg6) \
    os << prefix << #T "." #m << "(" << #arg1 << ", " << #arg2 << ", "        \
       << #arg3 << ", " << #arg4 << ", " << #arg5 << ", " << #arg6 << ")"     \
       << std::endl;
#define RPC_SNIPPETES10(prefix, ptr, T, m, arg1, arg2, arg3, arg4, arg5, arg6, \
                        arg7, arg8, arg9, arg10)                               \
    os << prefix << #T "." #m << "(" << #arg1 << ", " << #arg2 << ", "         \
       << #arg3 << ", " << #arg4 << ", " << #arg5 << ", " << #arg6 << ", "     \
       << #arg7 << ", " << #arg8 << ", " << #arg9 << ", " << #arg10 << ")"     \
       << std::endl;

#define Expand(...) __VA_ARGS__
using jsonrpccxx::GetHandle;
class JsonRpcService : public jsonrpccxx::JsonRpc2Server
{
    AuboApiPtr api_{ nullptr };

public:
    JsonRpcService(AuboApiPtr api)
    {
        api_ = api;

        JsonRpcServer::Add("getRobotNames",
                           GetHandle(&AuboApi::getRobotNames, *api_), {});

#define _INST(m, n, ...) Expand(RPC_BIND##n("", api_, m, __VA_ARGS__));
#define _FUNC(m, n, ...) Expand(RPC_BIND##n("", api_, m, __VA_ARGS__));
        SystemInfo_DECLARES;
        RuntimeMachine_DECLARES;
        RegisterControl_DECLARES;
        Math_DECLARES;
        Serial_DECLARES;
        Socket_DECLARES;
        SystemInfo_DECLARES;
#undef _INST
#undef _FUNC

        // 分别为不同的机器人绑定 RPC 函数, 以机器人的名字为前缀
#define _INST(m, n, ...) \
    Expand(RPC_BIND##n(robot_name + ".", robot, m, __VA_ARGS__));
#define _FUNC(m, n, ...) \
    Expand(RPC_BIND##n(robot_name + ".", robot, m, __VA_ARGS__));
        try {
            for (auto robot_name : api_->getRobotNames()) {
                auto robot = api_->getRobotInterface(robot_name);
                if (robot) {
                    IoControl_DECLARES;
                    MotionControl_DECLARES;
                    ForceControl_DECLARES;
                    RobotAlgorithm_DECLARES;
                    RobotConfig_DECLARES;
                    RobotManage_DECLARES;
                    RobotState_DECLARES;
                    SyncMove_DECLARES;
                    Trace_DECLARES;
                } else {
                    std::cout << "No Robot " << robot << ", " << robot_name
                              << std::endl;
                }
            }
        } catch (...) {
        }
#undef _INST
#undef _FUNC
    }

    static std::ostream &generateSnippets(std::ostream &os)
    {
        os << "getRobotNames()" << std::endl;
#define _INST(m, n, ...) Expand(RPC_SNIPPETES##n("", api_, m, __VA_ARGS__));
#define _FUNC(m, n, ...) Expand(RPC_SNIPPETES##n("", api_, m, __VA_ARGS__));
        SystemInfo_DECLARES;
        RuntimeMachine_DECLARES;
        RegisterControl_DECLARES;
        Math_DECLARES;
        Serial_DECLARES;
        Socket_DECLARES;
#undef _INST
#undef _FUNC
        // 分别为不同的机器人绑定 RPC 函数, 以机器人的名字为前缀

#define _INST(m, n, ...) \
    Expand(RPC_SNIPPETES##n("robot_name.", robot, m, __VA_ARGS__));
#define _FUNC(m, n, ...) \
    Expand(RPC_SNIPPETES##n("robot_name.", robot, m, __VA_ARGS__));

        IoControl_DECLARES;
        MotionControl_DECLARES;
        ForceControl_DECLARES;
        RobotAlgorithm_DECLARES;
        RobotConfig_DECLARES;
        RobotManage_DECLARES;
        RobotState_DECLARES;
        SyncMove_DECLARES;
        Trace_DECLARES;
#undef _INST
#undef _FUNC
        return os;
    }

    template <typename ReturnType, typename... ParamTypes>
    bool Add(const std::string &name,
             std::function<ReturnType(ParamTypes...)> f,
             const jsonrpccxx::NamedParamMapping &mapping =
                 jsonrpccxx::NAMED_PARAM_MAPPING)
    {
        if (name.rfind("rpc.", 0) == 0) {
            return false;
        }
        return dispatcher.Add(name, jsonrpccxx::methodHandle(f), mapping);
    }

    template <typename... ParamTypes>
    bool Add(const std::string &name, std::function<void(ParamTypes...)> f,
             const jsonrpccxx::NamedParamMapping &mapping =
                 jsonrpccxx::NAMED_PARAM_MAPPING)
    {
        if (name.rfind("rpc.", 0) == 0) {
            return false;
        }
        return dispatcher.Add(name, jsonrpccxx::notificationHandle(f), mapping);
    }
};

using JsonRpcServicePtr = std::shared_ptr<JsonRpcService>;
} // namespace common_interface
} // namespace arcs

#endif // AUBO_COMM_JSON_SERVICE_H
