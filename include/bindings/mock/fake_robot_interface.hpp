#ifndef ROBOTINTERFACEIMPL_H
#define ROBOTINTERFACEIMPL_H

#include <iostream>
#include <aubo/aubo_api.h>
//#include "./fakeit.hpp"

#include "bindings/function_traits.h"

namespace arcs {
namespace fake_robot {
using namespace common_interface;

template <typename T>
using return_type_t = typename function_traits<T>::return_type;

#define MOCK0(ClassType, method, ...)                                        \
    return_type_t<decltype(&ClassType::method)> method()                     \
    {                                                                        \
        return ReturnValue<return_type_t<decltype(&ClassType::method)>>()(); \
    }

#define MOCK1(ClassType, method, arg1, ...)                                  \
    return_type_t<decltype(&ClassType::method)> method(                      \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1)   \
    {                                                                        \
        return ReturnValue<return_type_t<decltype(&ClassType::method)>>()(); \
    }

#define MOCK2(ClassType, method, arg1, arg2, ...)                            \
    return_type_t<decltype(&ClassType::method)> method(                      \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2)   \
    {                                                                        \
        return ReturnValue<return_type_t<decltype(&ClassType::method)>>()(); \
    }

#define MOCK3(ClassType, method, arg1, arg2, arg3, ...)                      \
    return_type_t<decltype(&ClassType::method)> method(                      \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3)   \
    {                                                                        \
        return ReturnValue<return_type_t<decltype(&ClassType::method)>>()(); \
    }

#define MOCK4(ClassType, method, arg1, arg2, arg3, arg4, ...)                \
    return_type_t<decltype(&ClassType::method)> method(                      \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3,   \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4)   \
    {                                                                        \
        return ReturnValue<return_type_t<decltype(&ClassType::method)>>()(); \
    }

#define MOCK5(ClassType, method, arg1, arg2, arg3, arg4, arg5, ...)          \
    return_type_t<decltype(&ClassType::method)> method(                      \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3,   \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4,   \
        function_traits<decltype(&ClassType::method)>::args<4>::type arg5)   \
    {                                                                        \
        return ReturnValue<return_type_t<decltype(&ClassType::method)>>()(); \
    }

#define MOCK6(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6, ...)    \
    return_type_t<decltype(&ClassType::method)> method(                      \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,   \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,   \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3,   \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4,   \
        function_traits<decltype(&ClassType::method)>::args<4>::type arg5,   \
        function_traits<decltype(&ClassType::method)>::args<5>::type arg6)   \
    {                                                                        \
        return ReturnValue<return_type_t<decltype(&ClassType::method)>>()(); \
    }

#define EXTEND_CLASS(CLAZZ)                                              \
    class CLAZZ##Impl                                                    \
    {                                                                    \
    public:                                                              \
        CLAZZ##_DECLARES                                                 \
    };                                                                   \
    class CLAZZ##1 : public                                              \
                     CLAZZ{ public : CLAZZ##1(){ d_ = new CLAZZ##Impl(); \
    }                                                                    \
    ~CLAZZ##1() { delete (CLAZZ##Impl *)d_; }                            \
    }                                                                    \
    ;

#define _INST(m, n, ...) MOCK##n(m, __VA_ARGS__)
#define _FUNC(m, n, ...) MOCK##n(m, __VA_ARGS__)

EXTEND_CLASS(Math)
EXTEND_CLASS(Socket)
EXTEND_CLASS(Serial)
EXTEND_CLASS(SystemInfo)
EXTEND_CLASS(RuntimeMachine)
EXTEND_CLASS(RegisterControl)
EXTEND_CLASS(ForceControl)
EXTEND_CLASS(IoControl)
EXTEND_CLASS(MotionControl)
EXTEND_CLASS(RobotAlgorithm)
EXTEND_CLASS(RobotConfig)
EXTEND_CLASS(RobotManage)
EXTEND_CLASS(RobotState)
EXTEND_CLASS(SyncMove)
EXTEND_CLASS(Trace)

#undef _INST
#undef _FUNC

class RobotInterfaceImpl
{
public:
    RobotInterfaceImpl()
    {
        config_ = std::make_shared<RobotConfig1>();
        motion_control_ = std::make_shared<MotionControl1>();
        force_control_ = std::make_shared<ForceControl1>();
        io_control_ = std::make_shared<IoControl1>();
        sync_move_ = std::make_shared<SyncMove1>();
        alg_ = std::make_shared<RobotAlgorithm1>();
        manage_ = std::make_shared<RobotManage1>();
        robot_state_ = std::make_shared<RobotState1>();
        trace_ = std::make_shared<Trace1>();
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
class RobotInterface1 : public RobotInterface
{
public:
    RobotInterface1() { d_ = new RobotInterfaceImpl(); }
    ~RobotInterface1() { delete (RobotInterfaceImpl *)d_; }
};

class AuboApiImpl
{
public:
    AuboApiImpl()
    {
        robot_ = std::make_shared<RobotInterface1>();
        m_Math = std::make_shared<Math1>();
        m_Socket = std::make_shared<Socket1>();
        m_Serial = std::make_shared<Serial1>();
        m_RuntimeMachine = std::make_shared<RuntimeMachine1>();
        m_SystemInfo = std::make_shared<SystemInfo1>();
        m_RegisterControl = std::make_shared<RegisterControl1>();
    }

    MathPtr getMath() { return m_Math; }
    SocketPtr getSocket() { return m_Socket; }
    SerialPtr getSerial() { return m_Serial; }
    SystemInfoPtr getSystemInfo() { return m_SystemInfo; }
    RuntimeMachinePtr getRuntimeMachine() { return m_RuntimeMachine; }
    RegisterControlPtr getRegisterControl() { return m_RegisterControl; }
    std::vector<std::string> getRobotNames() { return { "robot1" }; }
    RobotInterfacePtr getRobotInterface(const std::string &name)
    {
        return robot_;
    }

private:
    friend class AuboApi;
    std::shared_ptr<RobotInterface> robot_;
    std::shared_ptr<Math> m_Math;
    std::shared_ptr<Socket> m_Socket;
    std::shared_ptr<Serial> m_Serial;
    std::shared_ptr<SystemInfo> m_SystemInfo;
    std::shared_ptr<RuntimeMachine> m_RuntimeMachine;
    std::shared_ptr<RegisterControl> m_RegisterControl;
};

class FakeAuboApi : public AuboApi
{
public:
    FakeAuboApi() { d_ = new AuboApiImpl; }
    ~FakeAuboApi() { delete (AuboApiImpl *)d_; }
};
} // namespace fake_robot
} // namespace arcs

#endif // ROBOTINTERFACEIMPL_H
