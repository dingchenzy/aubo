#ifndef COMMON_INTERFACE_PIMPL_H
#define COMMON_INTERFACE_PIMPL_H

#include "bindings/function_traits.h"

#define IMPL0(ClassType, method, ...)                               \
    return_type_t<decltype(&ClassType::method)> ClassType::method() \
    {                                                               \
        return ((ClassType##Impl *)d_)->method();                   \
    }

#define IMPL1(ClassType, method, arg1, ...)                                \
    return_type_t<decltype(&ClassType::method)> ClassType::method(         \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1) \
    {                                                                      \
        return ((ClassType##Impl *)d_)->method(arg1);                      \
    }

#define IMPL2(ClassType, method, arg1, arg2, ...)                          \
    return_type_t<decltype(&ClassType::method)> ClassType::method(         \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1, \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2) \
    {                                                                      \
        return ((ClassType##Impl *)d_)->method(arg1, arg2);                \
    }

#define IMPL3(ClassType, method, arg1, arg2, arg3, ...)                    \
    return_type_t<decltype(&ClassType::method)> ClassType::method(         \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1, \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2, \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3) \
    {                                                                      \
        return ((ClassType##Impl *)d_)->method(arg1, arg2, arg3);          \
    }

#define IMPL4(ClassType, method, arg1, arg2, arg3, arg4, ...)              \
    return_type_t<decltype(&ClassType::method)> ClassType::method(         \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1, \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2, \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3, \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4) \
    {                                                                      \
        return ((ClassType##Impl *)d_)->method(arg1, arg2, arg3, arg4);    \
    }

#define IMPL5(ClassType, method, arg1, arg2, arg3, arg4, arg5, ...)           \
    return_type_t<decltype(&ClassType::method)> ClassType::method(            \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1,    \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2,    \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3,    \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4,    \
        function_traits<decltype(&ClassType::method)>::args<4>::type arg5)    \
    {                                                                         \
        return ((ClassType##Impl *)d_)->method(arg1, arg2, arg3, arg4, arg5); \
    }

#define IMPL6(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6, ...)  \
    return_type_t<decltype(&ClassType::method)> ClassType::method(         \
        function_traits<decltype(&ClassType::method)>::args<0>::type arg1, \
        function_traits<decltype(&ClassType::method)>::args<1>::type arg2, \
        function_traits<decltype(&ClassType::method)>::args<2>::type arg3, \
        function_traits<decltype(&ClassType::method)>::args<3>::type arg4, \
        function_traits<decltype(&ClassType::method)>::args<4>::type arg5, \
        function_traits<decltype(&ClassType::method)>::args<5>::type arg6) \
    {                                                                      \
        return ((ClassType##Impl *)d_)                                     \
            ->method(arg1, arg2, arg3, arg4, arg5, arg6);                  \
    }

/// 定义构造和析构函数
#define DEFINE_CTOR(CLAZZ) \
    CLAZZ::CLAZZ() {}      \
    CLAZZ::~CLAZZ() {}

#define IMPL_ALL                 \
    DEFINE_CTOR(Trace)           \
    DEFINE_CTOR(Math)            \
    DEFINE_CTOR(Socket)          \
    DEFINE_CTOR(Serial)          \
    DEFINE_CTOR(SystemInfo)      \
    DEFINE_CTOR(RuntimeMachine)  \
    DEFINE_CTOR(RegisterControl) \
    DEFINE_CTOR(SyncMove)        \
    DEFINE_CTOR(RobotState)      \
    DEFINE_CTOR(MotionControl)   \
    DEFINE_CTOR(AuboApi)         \
    DEFINE_CTOR(RobotInterface)  \
    DEFINE_CTOR(ForceControl)    \
    DEFINE_CTOR(IoControl)       \
    DEFINE_CTOR(RobotConfig)     \
    DEFINE_CTOR(RobotAlgorithm)  \
    DEFINE_CTOR(RobotManage)     \
    Trace_DECLARES;              \
    Math_DECLARES;               \
    Socket_DECLARES;             \
    Serial_DECLARES;             \
    SystemInfo_DECLARES;         \
    RuntimeMachine_DECLARES;     \
    SyncMove_DECLARES;           \
    RobotState_DECLARES;         \
    MotionControl_DECLARES;      \
    AuboApi_DECLARES;            \
    ForceControl_DECLARES;       \
    RegisterControl_DECLARES;    \
    IoControl_DECLARES;          \
    RobotInterface_DECLARES;     \
    RobotConfig_DECLARES;        \
    RobotAlgorithm_DECLARES;     \
    RobotManage_DECLARES;

//#define X(m, n, f, ...) IMPL##n(m, f, __VA_ARGS__)
//#undef X

#endif
