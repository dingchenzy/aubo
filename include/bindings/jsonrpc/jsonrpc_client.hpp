#ifndef COMMON_INTERFACE_JSONRPC_CLIENT_H
#define COMMON_INTERFACE_JSONRPC_CLIENT_H

#include <mutex>
#include <nlohmann/json.hpp>
#include <jsonrpccxx/client.hpp>
#include <aubo/aubo_api.h>

#include "bindings/function_traits.h"
#include <bindings/jsonrpc/jsonrpccxx_ex.h>

namespace arcs {
namespace jsonrpc_client {
using namespace common_interface;

#define M0(ClassType, method, ...)                                           \
    function_traits<decltype(&ClassType::method)>::return_type method()      \
                                                                             \
    {                                                                        \
        if (fake_) {                                                         \
            return ReturnValue<                                              \
                return_type_t<decltype(&ClassType::method)>>()();            \
        }                                                                    \
        try {                                                                \
            return client_.CallMethodNamed<                                  \
                function_traits<decltype(&ClassType::method)>::return_type>( \
                jsonrpccxx::get_uuid(), _getName(#method), {});              \
        } catch (jsonrpccxx::JsonRpcException & e) {                         \
            throw AuboException(e.Code(), #method, e.what());                \
        } catch (std::exception & e) {                                       \
            throw AuboException(-1, #method, e.what());                      \
        } catch (...) {                                                      \
            throw AuboException(-1, #method, "Unkown exception");            \
        }                                                                    \
    }

#define M1(ClassType, method, arg1)                                          \
    function_traits<decltype(&ClassType::method)>::return_type method(       \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1)   \
                                                                             \
    {                                                                        \
        try {                                                                \
            if (fake_) {                                                     \
                return ReturnValue<                                          \
                    return_type_t<decltype(&ClassType::method)>>()();        \
            }                                                                \
            return client_.CallMethodNamed<                                  \
                function_traits<decltype(&ClassType::method)>::return_type>( \
                jsonrpccxx::get_uuid(), _getName(#method),                   \
                { { #arg1, arg1 } });                                        \
        } catch (jsonrpccxx::JsonRpcException & e) {                         \
            throw AuboException(e.Code(), #method, e.what());                \
        } catch (std::exception & e) {                                       \
            throw AuboException(-1, #method, e.what());                      \
        } catch (...) {                                                      \
            throw AuboException(-1, #method, "Unkown exception");            \
        }                                                                    \
    }

#define M2(ClassType, method, arg1, arg2)                                    \
    function_traits<decltype(&ClassType::method)>::return_type method(       \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2)   \
                                                                             \
    {                                                                        \
        try {                                                                \
            if (fake_) {                                                     \
                return ReturnValue<                                          \
                    return_type_t<decltype(&ClassType::method)>>()();        \
            }                                                                \
            return client_.CallMethodNamed<                                  \
                function_traits<decltype(&ClassType::method)>::return_type>( \
                jsonrpccxx::get_uuid(), _getName(#method),                   \
                { { #arg1, arg1 }, { #arg2, arg2 } });                       \
        } catch (jsonrpccxx::JsonRpcException & e) {                         \
            throw AuboException(e.Code(), #method, e.what());                \
        } catch (std::exception & e) {                                       \
            throw AuboException(-1, #method, e.what());                      \
        } catch (...) {                                                      \
            throw AuboException(-1, #method, "Unkown exception");            \
        }                                                                    \
    }

#define M3(ClassType, method, arg1, arg2, arg3)                              \
    function_traits<decltype(&ClassType::method)>::return_type method(       \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3)   \
                                                                             \
    {                                                                        \
        try {                                                                \
            if (fake_) {                                                     \
                return ReturnValue<                                          \
                    return_type_t<decltype(&ClassType::method)>>()();        \
            }                                                                \
            return client_.CallMethodNamed<                                  \
                function_traits<decltype(&ClassType::method)>::return_type>( \
                jsonrpccxx::get_uuid(), _getName(#method),                   \
                { { #arg1, arg1 }, { #arg2, arg2 }, { #arg3, arg3 } });      \
        } catch (jsonrpccxx::JsonRpcException & e) {                         \
            throw AuboException(e.Code(), #method, e.what());                \
        } catch (std::exception & e) {                                       \
            throw AuboException(-1, #method, e.what());                      \
        } catch (...) {                                                      \
            throw AuboException(-1, #method, "Unkown exception");            \
        }                                                                    \
    }

#define M4(ClassType, method, arg1, arg2, arg3, arg4)                        \
    function_traits<decltype(&ClassType::method)>::return_type method(       \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3,   \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4)   \
                                                                             \
    {                                                                        \
        try {                                                                \
            if (fake_) {                                                     \
                return ReturnValue<                                          \
                    return_type_t<decltype(&ClassType::method)>>()();        \
            }                                                                \
            return client_.CallMethodNamed<                                  \
                function_traits<decltype(&ClassType::method)>::return_type>( \
                jsonrpccxx::get_uuid(), _getName(#method),                   \
                { { #arg1, arg1 },                                           \
                  { #arg2, arg2 },                                           \
                  { #arg3, arg3 },                                           \
                  { #arg4, arg4 } });                                        \
        } catch (jsonrpccxx::JsonRpcException & e) {                         \
            throw AuboException(e.Code(), #method, e.what());                \
        } catch (std::exception & e) {                                       \
            throw AuboException(-1, #method, e.what());                      \
        } catch (...) {                                                      \
            throw AuboException(-1, #method, "Unkown exception");            \
        }                                                                    \
    }

#define M5(ClassType, method, arg1, arg2, arg3, arg4, arg5)                  \
    function_traits<decltype(&ClassType::method)>::return_type method(       \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3,   \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4,   \
        function_traits<decltype(&ClassType::method)>::args<4>::type arg5)   \
                                                                             \
    {                                                                        \
        try {                                                                \
            if (fake_) {                                                     \
                return ReturnValue<                                          \
                    return_type_t<decltype(&ClassType::method)>>()();        \
            }                                                                \
            return client_.CallMethodNamed<                                  \
                function_traits<decltype(&ClassType::method)>::return_type>( \
                jsonrpccxx::get_uuid(), _getName(#method),                   \
                { { #arg1, arg1 },                                           \
                  { #arg2, arg2 },                                           \
                  { #arg3, arg3 },                                           \
                  { #arg4, arg4 },                                           \
                  { #arg5, arg5 } });                                        \
        } catch (jsonrpccxx::JsonRpcException & e) {                         \
            throw AuboException(e.Code(), #method, e.what());                \
        } catch (std::exception & e) {                                       \
            throw AuboException(-1, #method, e.what());                      \
        } catch (...) {                                                      \
            throw AuboException(-1, #method, "Unkown exception");            \
        }                                                                    \
    }

#define M6(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6)            \
    function_traits<decltype(&ClassType::method)>::return_type method(       \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3,   \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4,   \
        function_traits<decltype(&ClassType::method)>::args<4>::type arg5,   \
        function_traits<decltype(&ClassType::method)>::args<5>::type arg6)   \
                                                                             \
    {                                                                        \
        try {                                                                \
            if (fake_) {                                                     \
                return ReturnValue<                                          \
                    return_type_t<decltype(&ClassType::method)>>()();        \
            }                                                                \
            return client_.CallMethodNamed<                                  \
                function_traits<decltype(&ClassType::method)>::return_type>( \
                jsonrpccxx::get_uuid(), _getName(#method),                   \
                { { #arg1, arg1 },                                           \
                  { #arg2, arg2 },                                           \
                  { #arg3, arg3 },                                           \
                  { #arg4, arg4 },                                           \
                  { #arg5, arg5 },                                           \
                  { #arg6, arg6 } });                                        \
        } catch (jsonrpccxx::JsonRpcException & e) {                         \
            throw AuboException(e.Code(), #method, e.what());                \
        } catch (std::exception & e) {                                       \
            throw AuboException(-1, #method, e.what());                      \
        } catch (...) {                                                      \
            throw AuboException(-1, #method, "Unkown exception");            \
        }                                                                    \
    }

#define EXTEND_CLASS(CLAZZ)                                               \
    class CLAZZ##1                                                        \
        : public CLAZZ{ public : CLAZZ##1(                                \
              const std::string &name, jsonrpccxx::JsonRpcClient &client, \
              bool fake){ d_ = new CLAZZ##Impl(name, client, fake);       \
    }                                                                     \
    ~CLAZZ##1() { delete (CLAZZ##Impl *)d_; }                             \
    }                                                                     \
    ;

#define Expand(...)      __VA_ARGS__
#define _INST(m, n, ...) Expand(M##n(m, __VA_ARGS__))
#define _FUNC(m, n, ...) Expand(M##n(m, __VA_ARGS__))

#define MM(m)                                                                 \
    class m##Impl                                                             \
    {                                                                         \
        jsonrpccxx::JsonRpcClient &client_;                                   \
        bool fake_{ false };                                                  \
        friend class m;                                                       \
                                                                              \
    public:                                                                   \
        std::string _getName(const char *f)                                   \
        {                                                                     \
            return std::string(#m ".") + f;                                   \
        }                                                                     \
        m##Impl(jsonrpccxx::JsonRpcClient &client, bool fake = false)         \
            : client_(client), fake_(fake)                                    \
        {                                                                     \
        }                                                                     \
        ~m##Impl() {}                                                         \
        m##_DECLARES;                                                         \
    };                                                                        \
                                                                              \
    class m##1 : public                                                       \
                 m{ public : m##1(jsonrpccxx::JsonRpcClient & client,         \
                                  bool fake){ d_ = new m##Impl(client, fake); \
    }                                                                         \
    ~m##1() { delete (m##Impl *)d_; }                                         \
    }                                                                         \
    ;

#define MM1(m)                                                                 \
    class m##Impl                                                              \
    {                                                                          \
        jsonrpccxx::JsonRpcClient &client_;                                    \
        std::string name_;                                                     \
        bool fake_{ false };                                                   \
        friend class m;                                                        \
                                                                               \
    public:                                                                    \
        std::string _getName(const char *f) { return name_ + "." #m "." + f; } \
        m##Impl(const std::string &name, jsonrpccxx::JsonRpcClient &client,    \
                bool fake = false)                                             \
            : client_(client), name_(name), fake_(fake)                        \
        {                                                                      \
        }                                                                      \
        ~m##Impl() {}                                                          \
        m##_DECLARES;                                                          \
    };                                                                         \
                                                                               \
    class m##1                                                                 \
        : public m{ public : m##1(                                             \
              const std::string &name, jsonrpccxx::JsonRpcClient &client,      \
              bool fake){ d_ = new m##Impl(name, client, fake);                \
    }                                                                          \
    ~m##1() { delete (m##Impl *)d_; }                                          \
    }                                                                          \
    ;

MM(Math)
MM(Serial)
MM(Socket)
MM(SystemInfo)
MM(RuntimeMachine)
MM(RegisterControl)

MM1(ForceControl)
MM1(IoControl)
MM1(MotionControl)
MM1(RobotAlgorithm)
MM1(RobotConfig)
MM1(RobotManage)
MM1(RobotState)
MM1(SyncMove)
MM1(Trace)

#undef _INST
#undef _FUNC

class RobotInterfaceImpl
{
public:
    RobotInterfaceImpl(const std::string &name,
                       jsonrpccxx::JsonRpcClient &client, bool fake = false)
        : name_(name), client_(client), fake_(fake)
    {
        config_ = std::make_shared<RobotConfig1>(name_, client_, fake_);
        motion_control_ =
            std::make_shared<MotionControl1>(name_, client_, fake_);
        force_control_ = std::make_shared<ForceControl1>(name_, client_, fake_);
        io_control_ = std::make_shared<IoControl1>(name_, client_, fake_);
        sync_move_ = std::make_shared<SyncMove1>(name_, client_, fake_);
        alg_ = std::make_shared<RobotAlgorithm1>(name_, client_, fake_);
        manage_ = std::make_shared<RobotManage1>(name_, client_, fake_);
        robot_state_ = std::make_shared<RobotState1>(name_, client_, fake_);
        trace_ = std::make_shared<Trace1>(name_, client_, fake_);
    }

    ~RobotInterfaceImpl() {}

    RobotConfigPtr getRobotConfig() { return config_; }
    MotionControlPtr getMotionControl() { return motion_control_; }
    ForceControlPtr getForceControl() { return force_control_; }
    IoControlPtr getIoControl() { return io_control_; }
    SyncMovePtr getSyncMove() { return sync_move_; }
    RobotAlgorithmPtr getRobotAlgorithm() { return alg_; }
    RobotManagePtr getRobotManage() { return manage_; }
    RobotStatePtr getRobotState() { return robot_state_; }
    TracePtr getTrace() { return trace_; }

private:
    friend class RobotInterface;
    std::string name_;
    jsonrpccxx::JsonRpcClient &client_;
    bool fake_{ false };

    std::shared_ptr<RobotConfig1> config_;
    std::shared_ptr<MotionControl1> motion_control_;
    std::shared_ptr<ForceControl1> force_control_;
    std::shared_ptr<IoControl1> io_control_;
    std::shared_ptr<SyncMove1> sync_move_;
    std::shared_ptr<RobotAlgorithm1> alg_;
    std::shared_ptr<RobotManage1> manage_;
    std::shared_ptr<RobotState1> robot_state_;
    std::shared_ptr<RuntimeMachine1> runtime_machine_;
    std::shared_ptr<Trace1> trace_;
};

EXTEND_CLASS(RobotInterface)
using RobotInterface1Ptr = std::shared_ptr<RobotInterface1>;

class AuboApiImpl
{
public:
    AuboApiImpl(std::shared_ptr<jsonrpccxx::IClientConnector> connector,
                bool fake = false)
        : connector_(connector), client_(*connector, jsonrpccxx::version::v2),
          fake_(fake)
    {
        math_ = std::make_shared<Math1>(client_, fake);
        socket_ = std::make_shared<Socket1>(client_, fake);
        serial_ = std::make_shared<Serial1>(client_, fake);
        system_info_ = std::make_shared<SystemInfo1>(client_, fake);
        runtime_machine_ = std::make_shared<RuntimeMachine1>(client_, fake);
        register_control_ = std::make_shared<RegisterControl1>(client_, fake);
    }

    int updateRobotNames()
    {
        std::unique_lock lck(mtx_);
        // 更新机器人名字列表
        robot_names_ = client_.CallMethodNamed<std::vector<std::string>>(
            jsonrpccxx::get_uuid(), "getRobotNames", {});

        for (auto &&name : robot_names_) {
            robot_interfaces_[name] =
                std::make_shared<RobotInterface1>(name, client_, fake_);
        }

        return 0;
    }

    int logout()
    {
        std::unique_lock lck(mtx_);
        robot_names_.clear();
        robot_interfaces_.clear();

        return 0;
    }

    MathPtr getMath() { return math_; }
    SocketPtr getSocket() { return socket_; }
    SerialPtr getSerial() { return serial_; }
    SystemInfoPtr getSystemInfo() { return system_info_; }
    RuntimeMachinePtr getRuntimeMachine() { return runtime_machine_; }
    RegisterControlPtr getRegisterControl() { return register_control_; }

    std::vector<std::string> getRobotNames()
    {
        if (fake_) {
            return { "robot1" };
        } else {
            return robot_names_;
        }
    }

    RobotInterfacePtr getRobotInterface(const std::string &name)
    {
        if (fake_) {
            if (!fake_robot_interface_) {
                fake_robot_interface_ =
                    std::make_shared<RobotInterface1>("robot1", client_, true);
            }
            return fake_robot_interface_;
        }
        std::unique_lock lck(mtx_);
        if (robot_interfaces_.find(name) != robot_interfaces_.end()) {
            return robot_interfaces_[name];
        }

        throw AuboException(-1, "No candidate robot names");
        return nullptr;
    }

    std::shared_ptr<jsonrpccxx::IClientConnector> getConnector()
    {
        return connector_;
    }

private:
    std::shared_ptr<jsonrpccxx::IClientConnector> connector_;
    jsonrpccxx::JsonRpcClient client_;
    bool fake_{ false };
    std::vector<std::string> robot_names_;
    MathPtr math_;
    SocketPtr socket_;
    SerialPtr serial_;
    SystemInfoPtr system_info_;
    RuntimeMachinePtr runtime_machine_;
    RegisterControlPtr register_control_;

    std::shared_ptr<RobotInterface> fake_robot_interface_{ nullptr };
    std::map<std::string, RobotInterface1Ptr> robot_interfaces_;
    std::mutex mtx_;
};

} // namespace jsonrpc_client
} // namespace arcs

#endif // COMMON_INTERFACE_JSONRPC_CLIENT_H
