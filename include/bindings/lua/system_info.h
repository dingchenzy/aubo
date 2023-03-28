#ifndef AUBO_SCRIPT_BINDING_LUA_SYSTEM_INFO_H
#define AUBO_SCRIPT_BINDING_LUA_SYSTEM_INFO_H

#include <iomanip>
#include "./utils.h"

namespace arcs {
namespace aubo_script {

class LuaSystemInfo
{
public:
    LuaSystemInfo(const SystemInfoPtr &_impl) : self(_impl) {}

    // 成员函数定义展开
#define _INST(m, n, f, ...) M##n(m, f, __VA_ARGS__)
#define _FUNC(m, n, f, ...) M##n(m, f, __VA_ARGS__)
    SystemInfo_DECLARES;
#undef _INST
#undef _FUNC

    static void registerLuaUserType(sol::state_view &L)
    {
        sol::automagic_enrollments enrollments;
        enrollments.default_constructor = false; // 不生成构造函数
        enrollments.destructor = false;
        auto u = L.new_usertype<LuaSystemInfo>("SystemInfo", enrollments);

#define _INST(m, n, f, ...)                \
    u.set_function(#f, &LuaSystemInfo::f); \
    wrapper_block1(L, "SystemInfo", #f);
#define _FUNC(m, n, f, ...)                \
    u.set_function(#f, &LuaSystemInfo::f); \
    wrapper_block2(L, "SystemInfo", #f);
        SystemInfo_DECLARES;
#undef _INST
#undef _FUNC
    }

    static std::ostream &generateTestScript(std::ostream &os,
                                            const char *prefix)
    {
#define _INST(m, n, f, ...) N##n(m, f, __VA_ARGS__)
#define _FUNC(m, n, f, ...) N##n(m, f, __VA_ARGS__)
        SystemInfo_DECLARES;
#undef _INST
#undef _FUNC

        return os;
    }

    static std::ostream &wrapperFunctions(std::ostream &os, const char *env,
                                          const char *prefix, int indent)
    {
#define _INST(m, n, f, ...) ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)
#define _FUNC(m, n, f, ...) ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)
        SystemInfo_DECLARES;
#undef _INST
#undef _FUNC
        return os;
    }

private:
    SystemInfoPtr self;
};
using LuaSystemInfoPtr = std::shared_ptr<LuaSystemInfo>;

} // namespace aubo_script
} // namespace arcs
#endif
