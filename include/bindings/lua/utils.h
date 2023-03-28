#ifndef AUBO_SCRIPT_BINDING_LUA_UTILS_H
#define AUBO_SCRIPT_BINDING_LUA_UTILS_H

#include <string.h>
#include <map>
#include <unordered_set>
#include <iomanip>
#include <sol/sol.hpp>

#include <aubo/aubo_api.h>
#include "bindings/function_traits.h"

namespace arcs {
namespace aubo_script {
using namespace common_interface;

inline std::ostream &operator<<(std::ostream &os, uint8_t vd)
{
    os << (int)vd;
    return os;
}

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &vd)
{
    os << "{ ";
    for (size_t i = 0; i < vd.size(); i++) {
        os << vd[i];
        if (i != (vd.size() - 1)) {
            os << ", ";
        }
    }
    os << " }";
    return os;
}

template <typename T, size_t N>
inline std::ostream &operator<<(std::ostream &os, const std::array<T, N> &vd)
{
    os << "{ ";
    for (size_t i = 0; i < vd.size(); i++) {
        os << vd[i];
        if (i != (vd.size() - 1)) {
            os << ", ";
        }
    }
    os << " }";
    return os;
}

template <typename T>
inline std::ostream &operator<<(std::ostream &os,
                                const std::vector<std::vector<T>> &vd)
{
    os << "{ ";
    for (size_t i = 0; i < vd.size(); i++) {
        os << vd[i];
        if (i != (vd.size() - 1)) {
            os << ", ";
        }
    }
    os << " }";
    return os;
}

template <typename K, typename V>
inline std::ostream &operator<<(std::ostream &os,
                                const std::unordered_map<K, V> &vd)
{
    os << "{ ";

    int n = 0;
    for (auto it = vd.begin(); it != vd.end(); it++) {
        os << "[" << it->first << "] = " << it->second;
        if (n != (vd.size() - 1)) {
            os << ", ";
        }
        n++;
    }
    os << " }";
    return os;
}

template <typename T>
inline std::ostream &operator<<(std::ostream &os,
                                const std::unordered_set<T> &vd)
{
    os << "{ ";

    int n = 0;
    for (auto it = vd.begin(); it != vd.end(); it++) {
        os << (*it);
        if (n != (vd.size() - 1)) {
            os << ", ";
        }
        n++;
    }
    os << " }";
    return os;
}

// 泛化版本
template <typename T>
struct type_convert : std::false_type
{
    typedef T type;
};
template <typename T>
struct type_convert<const T &> : std::false_type
{
    typedef typename type_convert<T>::type type;
};
template <typename T>
struct type_convert<const T> : std::false_type
{
    typedef typename type_convert<T>::type type;
};

template <typename T>
struct type_convert<std::vector<T>> : std::true_type
{
    typedef sol::nested<std::vector<T>> type;
};

template <typename T>
struct type_convert<const std::vector<T> &> : std::true_type
{
    typedef typename type_convert<std::vector<T>>::type type;
};

template <typename T, size_t N>
struct type_convert<std::array<T, N>> : std::true_type
{
    typedef sol::nested<std::array<T, N>> type;
};

template <typename T, size_t N>
struct type_convert<const std::array<T, N> &> : std::true_type
{
    typedef typename type_convert<std::array<T, N>>::type type;
};

template <>
struct type_convert<Box> : std::true_type
{
    typedef sol::nested<Box> type;
};

template <>
struct type_convert<const Box &> : std::true_type
{
    typedef typename type_convert<Box>::type type;
};

template <>
struct type_convert<Sphere> : std::true_type
{
    typedef sol::nested<Sphere> type;
};

template <>
struct type_convert<const Sphere &> : std::true_type
{
    typedef typename type_convert<Sphere>::type type;
};

template <>
struct type_convert<Cylinder> : std::true_type
{
    typedef sol::nested<Cylinder> type;
};

template <>
struct type_convert<const Cylinder &> : std::true_type
{
    typedef typename type_convert<Cylinder>::type type;
};

template <typename T>
struct type_convert<std::vector<std::vector<T>>> : std::true_type
{
    typedef sol::nested<std::vector<std::vector<T>>> type;
};

template <typename T>
struct type_convert<const std::vector<std::vector<T>> &> : std::true_type
{
    typedef typename type_convert<std::vector<std::vector<T>>>::type type;
};

template <typename T>
struct type_convert<std::unordered_set<T>> : std::true_type
{
    typedef sol::nested<std::unordered_set<T>> type;
};

template <typename T>
struct type_convert<const std::unordered_set<T> &> : std::true_type
{
    typedef typename type_convert<std::unordered_set<T>>::type type;
};

template <class _Key, class _Tp>
struct type_convert<std::unordered_map<_Key, _Tp>> : std::true_type
{
    typedef sol::nested<std::unordered_map<_Key, _Tp>> type;
};

template <class _Key, class _Tp>
struct type_convert<const std::unordered_map<_Key, _Tp> &> : std::true_type
{
    typedef typename type_convert<std::unordered_map<_Key, _Tp>>::type type;
};

// 特化 std::vector<double>
template struct type_convert<std::vector<double>>;
template struct type_convert<const std::vector<double> &>;
template struct type_convert<std::vector<int>>;
template struct type_convert<const std::vector<int> &>;
template struct type_convert<std::vector<uint8_t>>;
template struct type_convert<const std::vector<uint8_t> &>;
template struct type_convert<std::vector<bool>>;
template struct type_convert<const std::vector<bool> &>;
template struct type_convert<std::vector<JointStateType>>;
template struct type_convert<const std::vector<JointStateType> &>;
template struct type_convert<std::vector<RobotMsg>>;
template struct type_convert<const std::vector<RobotMsg> &>;
template struct type_convert<std::vector<JointServoModeType>>;
template struct type_convert<const std::vector<JointServoModeType> &>;
template struct type_convert<std::vector<std::string>>;
template struct type_convert<const std::vector<std::string> &>;
template struct type_convert<std::unordered_set<std::string>>;
template struct type_convert<const std::unordered_set<std::string> &>;
template struct type_convert<
    std::unordered_map<std::string, std::vector<double>>>;
template struct type_convert<
    const std::unordered_map<std::string, std::vector<double>> &>;
template struct type_convert<std::vector<std::vector<double>>>;
template struct type_convert<const std::vector<std::vector<double>> &>;

template <>
struct type_convert<std::tuple<std::string, std::vector<double>>>
    : std::true_type
{
    typedef std::tuple<std::string, sol::nested<std::vector<double>>> type;
};

template <>
struct type_convert<std::tuple<int, int, std::string>> : std::true_type
{
    typedef std::tuple<int, int, std::string> type;
};

template <>
struct type_convert<std::tuple<std::vector<double>, std::vector<double>>>
    : std::true_type
{
    typedef std::tuple<sol::nested<std::vector<double>>,
                       sol::nested<std::vector<double>>>
        type;
};

template <>
struct type_convert<std::tuple<std::vector<double>, int>> : std::true_type
{
    typedef std::tuple<sol::nested<std::vector<double>>, int> type;
};

template <>
struct type_convert<
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>
    : std::true_type
{
    typedef std::tuple<sol::nested<std::vector<double>>,
                       sol::nested<std::vector<double>>,
                       sol::nested<std::vector<double>>>
        type;
};

template <>
struct type_convert<std::tuple<std::vector<double>, std::vector<double>, double,
                               std::vector<double>>> : std::true_type
{
    typedef std::tuple<sol::nested<std::vector<double>>,
                       sol::nested<std::vector<double>>, double,
                       sol::nested<std::vector<double>>>
        type;
};

template <>
struct type_convert<std::tuple<double, std::vector<double>, std::vector<double>,
                               std::vector<double>>> : std::true_type
{
    typedef std::tuple<double, sol::nested<std::vector<double>>,
                       sol::nested<std::vector<double>>,
                       sol::nested<std::vector<double>>>
        type;
};

template <>
struct type_convert<std::vector<std::pair<uint32_t, uint32_t>>> : std::true_type
{
    typedef sol::nested<std::vector<std::pair<uint32_t, uint32_t>>> type;
};

template <typename T>
using type_convert_t = typename type_convert<T>::type;

template <typename T>
inline T getValue(const type_convert_t<T> &v)
{
    if constexpr (type_convert<T>()) {
        return v.value();
    } else {
        return v;
    }
}

template <typename T, typename RET = type_convert_t<T>>
inline RET toValue(const T &v)
{
    if constexpr (type_convert<T>()) {
        return sol::as_nested(v);
    } else {
        return v;
    }
}

template <>
inline std::tuple<std::string, sol::nested<std::vector<double>>>
toValue<std::tuple<std::string, std::vector<double>>>(
    const std::tuple<std::string, std::vector<double>> &v)
{
    return std::tuple(std::get<0>(v), sol::as_table(std::get<1>(v)));
}

template <>
inline std::tuple<sol::nested<std::vector<double>>,
                  sol::nested<std::vector<double>>>
toValue<std::tuple<std::vector<double>, std::vector<double>>>(
    const std::tuple<std::vector<double>, std::vector<double>> &v)
{
    return std::tuple(sol::as_table(std::get<0>(v)),
                      sol::as_table(std::get<1>(v)));
}

template <>
inline std::tuple<sol::nested<std::vector<double>>, int>
toValue<std::tuple<std::vector<double>, int>>(
    const std::tuple<std::vector<double>, int> &v)
{
    return std::tuple(sol::as_table(std::get<0>(v)), std::get<1>(v));
}
template <>
inline std::tuple<sol::nested<std::vector<double>>,
                  sol::nested<std::vector<double>>,
                  sol::nested<std::vector<double>>>
toValue<
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>(
    const std::tuple<std::vector<double>, std::vector<double>,
                     std::vector<double>> &v)
{
    return std::tuple(sol::as_table(std::get<0>(v)),
                      sol::as_table(std::get<1>(v)),
                      sol::as_table(std::get<2>(v)));
}

template <>
inline std::tuple<double, sol::nested<std::vector<double>>,
                  sol::nested<std::vector<double>>,
                  sol::nested<std::vector<double>>>
toValue<std::tuple<double, std::vector<double>, std::vector<double>,
                   std::vector<double>>>(
    const std::tuple<double, std::vector<double>, std::vector<double>,
                     std::vector<double>> &v)
{
    return std::tuple(std::get<0>(v), sol::as_table(std::get<1>(v)),
                      sol::as_table(std::get<2>(v)),
                      sol::as_table(std::get<3>(v)));
}

template <>
inline std::tuple<sol::nested<std::vector<double>>,
                  sol::nested<std::vector<double>>, double,
                  sol::nested<std::vector<double>>>
toValue<std::tuple<std::vector<double>, std::vector<double>, double,
                   std::vector<double>>>(
    const std::tuple<std::vector<double>, std::vector<double>, double,
                     std::vector<double>> &v)
{
    return std::tuple(sol::as_table(std::get<0>(v)),
                      sol::as_table(std::get<1>(v)), std::get<2>(v),
                      sol::as_table(std::get<3>(v)));
}

#define M0(ClassType, method, ...)                                             \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method()                                                                   \
    {                                                                          \
        return toValue<                                                        \
            function_traits<decltype(&ClassType::method)>::return_type>(       \
            self->method());                                                   \
    }

#define M1(ClassType, method, arg1, ...)                                       \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method(type_convert_t<                                                     \
           function_traits<decltype(&ClassType::method)>::args<0>::type>       \
               arg1)                                                           \
    {                                                                          \
        return toValue<                                                        \
            function_traits<decltype(&ClassType::method)>::return_type>(       \
            self->method(getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<0>::type>(arg1)));     \
    }

#define M2(ClassType, method, arg1, arg2, ...)                                 \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method(type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<0>::type>   \
               arg1,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<1>::type>   \
               arg2)                                                           \
    {                                                                          \
        return toValue<                                                        \
            function_traits<decltype(&ClassType::method)>::return_type>(       \
            self->method(getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<0>::type>(arg1),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<1>::type>(arg2)));     \
    }

#define M3(ClassType, method, arg1, arg2, arg3, ...)                           \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method(type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<0>::type>   \
               arg1,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<1>::type>   \
               arg2,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<2>::type>   \
               arg3)                                                           \
    {                                                                          \
        return toValue<                                                        \
            function_traits<decltype(&ClassType::method)>::return_type>(       \
            self->method(getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<0>::type>(arg1),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<1>::type>(arg2),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<2>::type>(arg3)));     \
    }

#define M4(ClassType, method, arg1, arg2, arg3, arg4, ...)                     \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method(type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<0>::type>   \
               arg1,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<1>::type>   \
               arg2,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<2>::type>   \
               arg3,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<3>::type>   \
               arg4)                                                           \
    {                                                                          \
        return toValue<                                                        \
            function_traits<decltype(&ClassType::method)>::return_type>(       \
            self->method(getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<0>::type>(arg1),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<1>::type>(arg2),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<2>::type>(arg3),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<3>::type>(arg4)));     \
    }

#define M5(ClassType, method, arg1, arg2, arg3, arg4, arg5, ...)               \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method(type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<0>::type>   \
               arg1,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<1>::type>   \
               arg2,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<2>::type>   \
               arg3,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<3>::type>   \
               arg4,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<4>::type>   \
               arg5)                                                           \
    {                                                                          \
        return toValue<                                                        \
            function_traits<decltype(&ClassType::method)>::return_type>(       \
            self->method(getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<0>::type>(arg1),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<1>::type>(arg2),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<2>::type>(arg3),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<3>::type>(arg4),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<4>::type>(arg5)));     \
    }

#define M6(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6, ...)         \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method(type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<0>::type>   \
               arg1,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<1>::type>   \
               arg2,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<2>::type>   \
               arg3,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<3>::type>   \
               arg4,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<4>::type>   \
               arg5,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<5>::type>   \
               arg6)                                                           \
    {                                                                          \
        return toValue<                                                        \
            function_traits<decltype(&ClassType::method)>::return_type>(       \
            self->method(getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<0>::type>(arg1),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<1>::type>(arg2),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<2>::type>(arg3),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<3>::type>(arg4),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<4>::type>(arg5),       \
                         getValue<function_traits<decltype(                    \
                             &ClassType::method)>::args<5>::type>(arg6)));     \
    }

#define M10(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, \
            arg9, arg10, ...)                                                  \
    type_convert_t<function_traits<decltype(&ClassType::method)>::return_type> \
    method(type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<0>::type>   \
               arg1,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<1>::type>   \
               arg2,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<2>::type>   \
               arg3,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<3>::type>   \
               arg4,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<4>::type>   \
               arg5,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<5>::type>   \
               arg6,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<6>::type>   \
               arg7,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<7>::type>   \
               arg8,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<8>::type>   \
               arg9,                                                           \
           type_convert_t<                                                     \
               function_traits<decltype(&ClassType::method)>::args<9>::type>   \
               arg10)                                                          \
    {                                                                          \
        return toValue<function_traits<decltype(&ClassType::method)>::         \
                           return_type>(self->method(                          \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<0>::type>( \
                arg1),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<1>::type>( \
                arg2),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<2>::type>( \
                arg3),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<3>::type>( \
                arg4),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<4>::type>( \
                arg5),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<5>::type>( \
                arg6),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<6>::type>( \
                arg7),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<7>::type>( \
                arg8),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<8>::type>( \
                arg9),                                                         \
            getValue<                                                          \
                function_traits<decltype(&ClassType::method)>::args<9>::type>( \
                arg10)));                                                      \
    }

#define N0(ClassType, method, ...) \
    os << prefix << "get" #ClassType "():" #method "() " << std::endl;
#define N1(ClassType, method, ...)                     \
    os << prefix << "get" #ClassType "():" #method "(" \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<0>::type>()() \
       << ") " << std::endl;
#define N2(ClassType, method, ...)                     \
    os << prefix << "get" #ClassType "():" #method "(" \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<0>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<1>::type>()() \
       << ") " << std::endl;
#define N3(ClassType, method, ...)                     \
    os << prefix << "get" #ClassType "():" #method "(" \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<0>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<1>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<2>::type>()() \
       << ") " << std::endl;
#define N4(ClassType, method, ...)                     \
    os << prefix << "get" #ClassType "():" #method "(" \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<0>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<1>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<2>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<3>::type>()() \
       << ") " << std::endl;
#define N5(ClassType, method, ...)                     \
    os << prefix << "get" #ClassType "():" #method "(" \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<0>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<1>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<2>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<3>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<4>::type>()() \
       << ") " << std::endl;
#define N6(ClassType, method, ...)                     \
    os << prefix << "get" #ClassType "():" #method "(" \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<0>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<1>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<2>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<3>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<4>::type>()() \
       << ","                                          \
       << ReturnValue<function_traits<decltype(        \
              &ClassType::method)>::args<5>::type>()() \
       << ") " << std::endl;

#define ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)        \
    os << std::setw(indent * 2) << ""                        \
       << "function " << env << "." #f "(...)" << std::endl; \
    os << std::setw(indent * 2 + 2) << ""                    \
       << "return " << prefix << #f "(...) " << std::endl;   \
    os << std::setw(indent * 2) << ""                        \
       << "end" << std::endl;

inline void wrapper_block(sol::state_view &L, const char *m, const char *f)
{
    sol::protected_function wrapper_block = L.safe_script(
        R"(return function(f, name)
    local tmp = f
    local unpack = unpack or table.unpack
    local sched = require('aubo.scheduler')
    return function(...)
        local arg = {...}
        local res = 0
        repeat
            res = tmp(unpack(arg))
            sched.sync()
        until res <= 0
        if res < 0 then
            sched.error = true
        end
    end
end)",
        [](lua_State *, sol::protected_function_result pfr) {
            // pfr will contain things that went wrong, for either loading or
            // executing the script the user can do whatever they like here,
            // including throw. Otherwise...
            sol::error err = pfr;
            std::cout << "An error (an expected one) occurred: " << err.what()
                      << std::endl;

            // ... they need to return the protected_function_result
            return pfr;
        },
        "wrapper_block");

    L[m][f] = wrapper_block(L[m][f], f);
};

// 与 wrapper_block 的区别在于如果指令执行成功，不会附带一个 sync， 会直接退出
inline void wrapper_block1(sol::state_view &L, const char *m, const char *f)
{
    sol::protected_function wrapper_block = L.safe_script(
        R"(return function(f, name)
    local tmp = f
    local unpack = unpack or table.unpack
    local sched = require('aubo.scheduler')
    return function(...)
        local arg = {...}
        local res = tmp(unpack(arg))
        while (res > 0) do
            res = tmp(unpack(arg))
            sched.sync()
        end
        if res < 0 then
            sched.error = true
        end
    end
end)",
        [](lua_State *, sol::protected_function_result pfr) {
            // pfr will contain things that went wrong, for either loading or
            // executing the script the user can do whatever they like here,
            // including throw. Otherwise...
            sol::error err = pfr;
            std::cout << "An error (an expected one) occurred: " << err.what()
                      << std::endl;

            // ... they need to return the protected_function_result
            return pfr;
        },
        "wrapper_block1");

    L[m][f] = wrapper_block(L[m][f], f);
};

// 在原始指令之后增加 cancelpoint 避免死循环
inline void wrapper_block2(sol::state_view &L, const char *m, const char *f)
{
    sol::protected_function wrapper_block = L.safe_script(
        R"(return function(f, name)
    local tmp = f
    local unpack = unpack or table.unpack
    local sched = require('aubo.scheduler')
    return function(...)
        local arg = {...}
        local res = {tmp(unpack(arg))}
        sched.cancel_point()
        return unpack(res)
    end
end)",
        [](lua_State *, sol::protected_function_result pfr) {
            // pfr will contain things that went wrong, for either loading or
            // executing the script the user can do whatever they like here,
            // including throw. Otherwise...
            sol::error err = pfr;
            std::cout << "An error (an expected one) occurred: " << err.what()
                      << std::endl;

            // ... they need to return the protected_function_result
            return pfr;
        },
        "wrapper_block2");

    L[m][f] = wrapper_block(L[m][f], f);
};

} // namespace aubo_script
} // namespace arcs

#endif
