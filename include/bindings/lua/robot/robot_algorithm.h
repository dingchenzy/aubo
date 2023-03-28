#ifndef AUBO_SCRIPT_BINDING_LUA_ROBOT_ROBOT_ALGORITHM_H
#define AUBO_SCRIPT_BINDING_LUA_ROBOT_ROBOT_ALGORITHM_H

#include "../utils.h"

namespace arcs {
namespace aubo_script {

class LuaRobotAlgorithm
{
public:
    LuaRobotAlgorithm(const RobotAlgorithmPtr &_impl) : self(_impl) {}

    // 成员函数定义展开
#define _INST(m, n, f, ...) M##n(m, f, __VA_ARGS__)
#define _FUNC(m, n, f, ...) M##n(m, f, __VA_ARGS__)
    RobotAlgorithm_DECLARES;
#undef _FUNC
#undef _INST

    static void registerLuaUserType(sol::state_view &L)
    {
        sol::automagic_enrollments enrollments;
        enrollments.default_constructor = false; // 不生成构造函数
        enrollments.destructor = false;
        auto u =
            L.new_usertype<LuaRobotAlgorithm>("RobotAlgorithm", enrollments);

#define _INST(m, n, f, ...)                    \
    u.set_function(#f, &LuaRobotAlgorithm::f); \
    wrapper_block2(L, "RobotAlgorithm", #f);
#define _FUNC(m, n, f, ...)                    \
    u.set_function(#f, &LuaRobotAlgorithm::f); \
    wrapper_block2(L, "RobotAlgorithm", #f);
        RobotAlgorithm_DECLARES;
#undef _INST
#undef _FUNC
    }

    static std::ostream &generateTestScript(std::ostream &os,
                                            const char *prefix)
    {
#define _INST(m, n, f, ...) N##n(m, f, __VA_ARGS__)
#define _FUNC(m, n, f, ...) N##n(m, f, __VA_ARGS__)
        RobotAlgorithm_DECLARES;
#undef _INST
#undef _FUNC
        return os;
    }

    static std::ostream &wrapperFunctions(std::ostream &os, const char *env,
                                          const char *prefix, int indent)
    {
#define _INST(m, n, f, ...) ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)
#define _FUNC(m, n, f, ...) ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)
        RobotAlgorithm_DECLARES;
#undef _INST
#undef _FUNC
        return os;
    }

private:
    RobotAlgorithmPtr self;
};
using LuaRobotAlgorithmPtr = std::shared_ptr<LuaRobotAlgorithm>;

} // namespace aubo_script
} // namespace arcs
#endif
