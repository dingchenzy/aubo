#pragma once
#include <iostream>
#include <jsonrpccxx/server.hpp>

namespace jsonrpccxx {

template <typename T, typename ReturnType, typename... ParamTypes>
MethodHandle GetHandle(ReturnType (T::*method)(ParamTypes...) const,
                       T &instance)
{
    std::function<ReturnType(ParamTypes...)> function =
        [&instance, method](ParamTypes &&... params) -> ReturnType {
        return (instance.*method)(std::forward<ParamTypes>(params)...);
    };
    return GetHandle(function);
}

inline std::string get_uuid()
{
    static std::random_device dev;
    static std::mt19937 rng(dev());

    std::uniform_int_distribution<int> dist(0, 61);

    const char *v =
        "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const bool dash[] = { 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0 };

    std::stringstream ss;
    for (int i = 0; i < 8; i++) {
        if (dash[i])
            ss << "-";
        ss << v[dist(rng)];
        ss << v[dist(rng)];
    }
    return ss.str();
}
} // namespace jsonrpccxx
