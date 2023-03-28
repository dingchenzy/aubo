#ifndef AUBO_SCRIPT_BINDING_LUA_MATH_H
#define AUBO_SCRIPT_BINDING_LUA_MATH_H

#include <iomanip>
#include "./utils.h"

namespace arcs {
namespace aubo_script {

class LuaMath
{
public:
    LuaMath(const MathPtr &_impl) : self(_impl) {}

    // 成员函数定义展开
#define _FUNC(m, n, f, ...) M##n(m, f, __VA_ARGS__)
    Math_DECLARES;
#undef _FUNC

    static void registerLuaUserType(sol::state_view &L)
    {
        sol::automagic_enrollments enrollments;
        enrollments.default_constructor = false; // 不生成构造函数
        enrollments.destructor = false;
        auto u = L.new_usertype<LuaMath>("Math", enrollments);

#define _FUNC(m, n, f, ...)          \
    u.set_function(#f, &LuaMath::f); \
    wrapper_block2(L, "Math", #f);
        Math_DECLARES;
#undef _FUNC
    }

    static std::ostream &generateTestScript(std::ostream &os,
                                            const char *prefix)
    {
#define _FUNC(m, n, f, ...) N##n(m, f, __VA_ARGS__)
        Math_DECLARES;
#undef _FUNC

        return os;
    }

    static std::ostream &wrapperFunctions(std::ostream &os, const char *env,
                                          const char *prefix, int indent)
    {
#define _FUNC(m, n, f, ...) ROBOT_FUNC_WRAPPER(m, f, env, prefix, indent)
        Math_DECLARES;
#undef _FUNC
        return os;
    }

private:
    MathPtr self;
};
using LuaMathPtr = std::shared_ptr<LuaMath>;

} // namespace aubo_script
} // namespace arcs
#endif
