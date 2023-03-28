#ifndef AUBO_SCRIPT_BINDING_LUA_TRACE_H
#define AUBO_SCRIPT_BINDING_LUA_TRACE_H

#include "../utils.h"

namespace arcs {
namespace aubo_script {

class LuaTrace
{
public:
    LuaTrace(const TracePtr &_impl) : self(_impl) {}

    // 成员函数定义展开
#define _INST(m, n, f, ...) M##n(m, f, __VA_ARGS__)
#define _FUNC(m, n, f, ...) M##n(m, f, __VA_ARGS__)
    Trace_DECLARES;
#undef _INST
#undef _FUNC

    static void registerLuaUserType(sol::state_view &L)
    {
        uint64_t timestamp;
        TraceLevel level;
        int code;
        std::string source;
        std::vector<std::string> args;

        L.new_usertype<RobotMsg>("Trace",
                                 //
                                 "timestamp", &RobotMsg::timestamp,

                                 "level", &RobotMsg::level,

                                 "code", &RobotMsg::code,

                                 "source", &RobotMsg::source,

                                 "args", &RobotMsg::args);

        sol::automagic_enrollments enrollments;
        enrollments.default_constructor = false; // 不生成构造函数
        enrollments.destructor = false;
        auto u = L.new_usertype<LuaTrace>("Trace", enrollments);

#define _INST(m, n, f, ...)           \
    u.set_function(#f, &LuaTrace::f); \
    wrapper_block1(L, "Trace", #f);
#define _FUNC(m, n, f, ...)           \
    u.set_function(#f, &LuaTrace::f); \
    wrapper_block2(L, "Trace", #f);
        Trace_DECLARES;
#undef _INST
#undef _FUNC
    }

    static std::ostream &generateTestScript(std::ostream &os,
                                            const char *prefix)
    {
#define _INST(m, n, f, ...) N##n(m, f, __VA_ARGS__)
#define _FUNC(m, n, f, ...) N##n(m, f, __VA_ARGS__)
        Trace_DECLARES;
#undef _INST
#undef _FUNC
        return os;
    }

    static std::ostream &wrapperFunctions(std::ostream &os, const char *env,
                                          const char *prefix, int indent)
    {
#define _INST(m, n, f, ...) ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)
#define _FUNC(m, n, f, ...) ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)
        Trace_DECLARES;
#undef _INST
#undef _FUNC
        return os;
    }

private:
    TracePtr self;
};
using LuaTracePtr = std::shared_ptr<LuaTrace>;
} // namespace aubo_script
} // namespace arcs
#endif
