#ifndef COMMON_INTERFACE_FUNCTION_TRAITS_H
#define COMMON_INTERFACE_FUNCTION_TRAITS_H

#include <type_traits>
#include <functional>
#include <array>

#include <aubo/aubo_api.h>

// 获取宏参数个数(0~8)
// https://blog.csdn.net/tangchuanhui/article/details/99588304
#define ARGS_COUNT(...)                                          ARGS_COUNT_(0, ##__VA_ARGS__, NUM_SEQUENCE())
#define NUM_SEQUENCE()                                           8, 7, 6, 5, 4, 3, 2, 1, 0
#define ARGS_COUNT_(...)                                         ARGS_COUNT_N(__VA_ARGS__)
#define ARGS_COUNT_N(_1, _2, _3, _4, _5, _6, _7, _8, _9, N, ...) N

#define GET_VA_ARG_1(...)          GET_VA_ARG_1_(__VA_ARGS__, )
#define GET_VA_ARG_1_(a1, ...)     a1
#define GET_VA_ARG_2(...)          GET_VA_ARG_2_(__VA_ARGS__, )
#define GET_VA_ARG_2_(a1, a2, ...) a2

// https://stackoverflow.com/questions/33113740/decltype-of-member-functions
// https://www.cnblogs.com/qicosmos/p/4772328.html

namespace arcs {
namespace common_interface {
template <typename T>
struct function_traits;
//普通函数
template <typename Ret, typename... Args>
struct function_traits<Ret(Args...)>
{
public:
    enum
    {
        arity = sizeof...(Args)
    };
    typedef Ret function_type(Args...);
    typedef Ret return_type;
    using stl_function_type = std::function<function_type>;
    typedef Ret (*pointer)(Args...);

    template <size_t I>
    struct args
    {
        static_assert(
            I < arity,
            "index is out of range, index must less than sizeof Args");
        using type = typename std::tuple_element<I, std::tuple<Args...>>::type;
    };
};

//函数指针
template <typename Ret, typename... Args>
struct function_traits<Ret (*)(Args...)> : function_traits<Ret(Args...)>
{
};

// std::function
template <typename Ret, typename... Args>
struct function_traits<std::function<Ret(Args...)>>
    : function_traits<Ret(Args...)>
{
};

// member function
#define FUNCTION_TRAITS(...)                                               \
    template <typename ReturnType, typename ClassType, typename... Args>   \
    struct function_traits<ReturnType (ClassType::*)(Args...) __VA_ARGS__> \
        : function_traits<ReturnType(Args...)>                             \
    {                                                                      \
    };

FUNCTION_TRAITS()
FUNCTION_TRAITS(const)
FUNCTION_TRAITS(volatile)
FUNCTION_TRAITS(const volatile)

template <typename T>
using return_type_t = typename function_traits<T>::return_type;

//函数对象
template <typename Callable>
struct function_traits : function_traits<decltype(&Callable::operator())>
{
};

// 定义默认的返回值
template <typename T>
struct ReturnValue
{
};

template <typename T>
struct ReturnValue<const T &>
{
    T operator()() { return ReturnValue<T>()(); }
};
template <typename T>
struct ReturnValue<std::vector<T>>
{
    std::vector<T> operator()()
    {
        return std::vector<T>(6, ReturnValue<T>()());
    }
};
template <typename T>
struct ReturnValue<const std::vector<T> &>
{
    std::vector<T> operator()() { return ReturnValue<std::vector<T>>()(); }
};

#define TYPENAME(...) __VA_ARGS__
#define SETUP_RETURN_VALUE(T, ...)               \
    template <>                                  \
    struct ReturnValue<T>                        \
    {                                            \
        T operator()() { return __VA_ARGS__; }   \
    };                                           \
    template struct ReturnValue<std::vector<T>>; \
    template struct ReturnValue<const std::vector<T> &>;

SETUP_RETURN_VALUE(bool, true);
SETUP_RETURN_VALUE(int, 0);
SETUP_RETURN_VALUE(char, 0);
SETUP_RETURN_VALUE(uint8_t, 0);
SETUP_RETURN_VALUE(int16_t, 0);
SETUP_RETURN_VALUE(uint16_t, 0);
SETUP_RETURN_VALUE(uint32_t, 0);
SETUP_RETURN_VALUE(uint64_t, 0);
SETUP_RETURN_VALUE(int64_t, 0);
SETUP_RETURN_VALUE(float, 0.);
SETUP_RETURN_VALUE(double, 0.);
SETUP_RETURN_VALUE(std::string, "\"str\"");
SETUP_RETURN_VALUE(SafetyInputAction, SafetyInputAction::Unassigned);
SETUP_RETURN_VALUE(SafetyOutputRunState, SafetyOutputRunState::Unassigned);
SETUP_RETURN_VALUE(StandardInputAction, StandardInputAction::Default);
SETUP_RETURN_VALUE(StandardOutputRunState, StandardOutputRunState::None);
SETUP_RETURN_VALUE(OperationalModeType, OperationalModeType::Automatic);
SETUP_RETURN_VALUE(RobotControlModeType, RobotControlModeType::None);
SETUP_RETURN_VALUE(SafetyModeType, SafetyModeType::Undefined);
SETUP_RETURN_VALUE(RobotModeType, RobotModeType::NoController);
SETUP_RETURN_VALUE(JointStateType, JointStateType::Idle);
SETUP_RETURN_VALUE(JointServoModeType, JointServoModeType::None);
SETUP_RETURN_VALUE(RuntimeState, RuntimeState::Stopped);
SETUP_RETURN_VALUE(TaskFrameType, TaskFrameType::NONE);
SETUP_RETURN_VALUE(TraceLevel, TraceLevel::INFO);
SETUP_RETURN_VALUE(Box, {});
SETUP_RETURN_VALUE(Cylinder, {});
SETUP_RETURN_VALUE(Sphere, {});
SETUP_RETURN_VALUE(RobotSafetyParameterRange, {});
SETUP_RETURN_VALUE(CircleParameters, {});

SETUP_RETURN_VALUE(
    std::vector<RobotMsg>,
    std::vector<RobotMsg>(6, { 0, TraceLevel::INFO, 0, "src", {} }));
SETUP_RETURN_VALUE(TYPENAME(std::pair<uint32_t, uint32_t>),
                   std::pair<uint32_t, uint32_t>({ 0, 0 }));
SETUP_RETURN_VALUE(std::unordered_set<std::string>,
                   std::unordered_set<std::string>{ "\"s1\"", "\"s2\"" });
SETUP_RETURN_VALUE(ResultWithErrno,
                   ResultWithErrno({ std::vector<double>(6, 0.), 0 }));
SETUP_RETURN_VALUE(ResultWithErrno1,
                   ResultWithErrno1({ std::vector<std::vector<double>>(
                                          6, std::vector<double>(6, 0.)),
                                      0 }));
SETUP_RETURN_VALUE(TYPENAME(std::tuple<std::string, std::vector<double>>),
                   std::tuple<std::string, std::vector<double>>(
                       { "str", std::vector<double>(6, 0.) }));
SETUP_RETURN_VALUE(TYPENAME(std::tuple<std::string, double>),
                   std::tuple<std::string, double>({ "str", 0. }));
SETUP_RETURN_VALUE(TYPENAME(std::tuple<std::string, std::string>),
                   std::tuple<std::string, std::string>({ "str", "str" }));
SETUP_RETURN_VALUE(
    TYPENAME(std::unordered_map<std::string, std::vector<double>>),
    std::unordered_map<std::string, std::vector<double>>(
        { { "\"str\"", std::vector<double>(6, 0.) } }));
SETUP_RETURN_VALUE(
    TYPENAME(std::tuple<std::vector<double>, std::vector<double>>),
    std::tuple<std::vector<double>, std::vector<double>>(
        { std::vector<double>(6, 0.), std::vector<double>(6, 0.) }));
SETUP_RETURN_VALUE(TYPENAME(std::tuple<int, int, std::string>),
                   std::tuple<int, int, std::string>({ 0, 0, "str" }));
SETUP_RETURN_VALUE(Payload, Payload({ 0, std::vector<double>(6, 0.),
                                      std::vector<double>(6, 0.),
                                      std::vector<double>(6, 0.) }));
SETUP_RETURN_VALUE(
    TYPENAME(std::tuple<std::vector<double>, std::vector<double>,
                        std::vector<double>>),
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>(
        { std::vector<double>(6, 0.), std::vector<double>(6, 0.),
          std::vector<double>(6, 0.) }));

SETUP_RETURN_VALUE(TYPENAME(std::tuple<std::vector<double>, std::vector<double>,
                                       double, std::vector<double>>),
                   std::tuple<std::vector<double>, std::vector<double>, double,
                              std::vector<double>>(
                       { std::vector<double>(6, 0.), std::vector<double>(6, 0.),
                         0, std::vector<double>(6, 0.) }));
#undef SETUP_RETURN_VALUE

// 定义默认的传入参数
template <typename T>
struct ParamValue
{
};

template <typename T>
struct ParamValue<const T &>
{
    T operator()() { return ParamValue<T>()(); }
};

#define SETUP_PARAM_VALUE(T, ...)              \
    template <>                                \
    struct ParamValue<T>                       \
    {                                          \
        T operator()() { return __VA_ARGS__; } \
    };
SETUP_PARAM_VALUE(std::string, "str");
SETUP_PARAM_VALUE(int, 0);
SETUP_PARAM_VALUE(bool, true);
SETUP_PARAM_VALUE(double, 0.01);
SETUP_PARAM_VALUE(uint8_t, 0);
SETUP_PARAM_VALUE(uint32_t, 0);
SETUP_PARAM_VALUE(uint64_t, 0);
SETUP_PARAM_VALUE(int64_t, 0);
SETUP_PARAM_VALUE(std::vector<double>, std::vector<double>(6, 0.2));
SETUP_PARAM_VALUE(
    std::vector<std::vector<double>>,
    std::vector<std::vector<double>>(1, std::vector<double>(6, 0.2)));
SETUP_PARAM_VALUE(Box, { 0 });
SETUP_PARAM_VALUE(Sphere, { 0 });
SETUP_PARAM_VALUE(Cylinder, { 0 });
SETUP_PARAM_VALUE(std::vector<bool>, std::vector<bool>(6, true));
SETUP_PARAM_VALUE(TaskFrameType, TaskFrameType::FRAME_FORCE);
SETUP_PARAM_VALUE(SafetyInputAction, SafetyInputAction::Unassigned);
SETUP_PARAM_VALUE(SafetyOutputRunState, SafetyOutputRunState::Unassigned);
SETUP_PARAM_VALUE(StandardInputAction, StandardInputAction::Default);
SETUP_PARAM_VALUE(StandardOutputRunState, StandardOutputRunState::None);
SETUP_PARAM_VALUE(OperationalModeType, OperationalModeType::Automatic);
SETUP_PARAM_VALUE(RobotControlModeType, RobotControlModeType::None);
SETUP_PARAM_VALUE(SafetyModeType, SafetyModeType::Undefined);
SETUP_PARAM_VALUE(RobotModeType, RobotModeType::NoController);
SETUP_PARAM_VALUE(JointStateType, JointStateType::Idle);
SETUP_PARAM_VALUE(JointServoModeType, JointServoModeType::None);
SETUP_PARAM_VALUE(RuntimeState, RuntimeState::Stopped);
SETUP_PARAM_VALUE(TraceLevel, TraceLevel::INFO);
SETUP_PARAM_VALUE(
    TYPENAME(std::unordered_map<std::string, std::vector<double>>),
    std::unordered_map<std::string, std::vector<double>>(
        { { "str", std::vector<double>(6, 0.) } }));
SETUP_PARAM_VALUE(RobotSafetyParameterRange, {});
#undef TYPENAME
} // namespace common_interface
} // namespace arcs

#endif
