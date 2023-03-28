#ifndef AUBO_SCRIPT_BINDING_LUA_ROBOT_INTERFACE_H
#define AUBO_SCRIPT_BINDING_LUA_ROBOT_INTERFACE_H

#include <string.h>

#include "./math.h"
#include "./system_info.h"
#include "./runtime_machine.h"
#include "./robot/robot_config.h"
#include "./robot/force_control.h"
#include "./robot/motion_control.h"
#include "./robot/force_control.h"
#include "./robot/io_control.h"
#include "./robot/sync_move.h"
#include "./robot/robot_algorithm.h"
#include "./robot/robot_manage.h"
#include "./robot/robot_state.h"
#include "./robot/trace.h"

namespace arcs {
namespace aubo_script {

class LuaRobotInterface
{
public:
    LuaRobotInterface(const RobotInterfacePtr &_impl) : self(_impl)
    {
#define MEMBER_PTR(CLASS) \
    m_##CLASS = std::make_shared<Lua##CLASS>(self->get##CLASS());
        MEMBER_PTR(RobotConfig);
        MEMBER_PTR(MotionControl);
        MEMBER_PTR(ForceControl);
        MEMBER_PTR(IoControl);
        MEMBER_PTR(SyncMove);
        MEMBER_PTR(RobotAlgorithm);
        MEMBER_PTR(RobotManage);
        MEMBER_PTR(RobotState);
        MEMBER_PTR(Trace);
#undef MEMBER_PTR
    }

#define MEMBER_FUNC(CLASS)     \
private:                       \
    Lua##CLASS##Ptr m_##CLASS; \
                               \
public:                        \
    Lua##CLASS##Ptr get##CLASS() { return m_##CLASS; }

    MEMBER_FUNC(RobotConfig);
    MEMBER_FUNC(MotionControl);
    MEMBER_FUNC(ForceControl);
    MEMBER_FUNC(IoControl);
    MEMBER_FUNC(SyncMove);
    MEMBER_FUNC(RobotAlgorithm);
    MEMBER_FUNC(RobotManage);
    MEMBER_FUNC(RobotState);
    MEMBER_FUNC(SystemInfo);
    MEMBER_FUNC(RuntimeMachine);
    MEMBER_FUNC(Trace);
#undef MEMBER_FUNC

    static void registerLuaUserType(sol::state_view &L)
    {
        LuaRobotConfig::registerLuaUserType(L);
        LuaIoControl::registerLuaUserType(L);
        LuaMotionControl::registerLuaUserType(L);
        LuaRobotAlgorithm::registerLuaUserType(L);
        LuaForceControl::registerLuaUserType(L);
        LuaRobotManage::registerLuaUserType(L);
        LuaRobotState::registerLuaUserType(L);
        LuaSyncMove::registerLuaUserType(L);
        LuaTrace::registerLuaUserType(L);

        sol::automagic_enrollments enrollments;
        enrollments.default_constructor = false; // 不生成构造函数
        enrollments.destructor = false;
        auto u =
            L.new_usertype<LuaRobotInterface>("RobotInterface", enrollments);

#define _INST(_, n, f, ...) u.set_function(#f, &LuaRobotInterface::f);
#define _FUNC(_, n, f, ...) u.set_function(#f, &LuaRobotInterface::f);
        RobotInterface_DECLARES
#undef _INST
#undef _FUNC
    }

    static std::ostream &generateTestScript(std::ostream &os,
                                            const char *prefix)
    {
        static char p[100];
        sprintf(p, "%s%s", prefix, "getRobotInterface(\"\"):");

        LuaRobotConfig::generateTestScript(os, p);
        LuaIoControl::generateTestScript(os, p);
        LuaMotionControl::generateTestScript(os, p);
        LuaRobotAlgorithm::generateTestScript(os, p);
        LuaForceControl::generateTestScript(os, p);
        LuaRobotManage::generateTestScript(os, p);
        LuaRobotState::generateTestScript(os, p);
        LuaSyncMove::generateTestScript(os, p);
        LuaTrace::generateTestScript(os, p);

        return os;
    }

    static std::ostream &wrapperFunctions(std::ostream &os, int indent)
    {
        LuaRobotConfig::wrapperFunctions(os, "env", "robotConfig:", indent);
        LuaIoControl::wrapperFunctions(os, "env", "ioControl:", indent);
        LuaMotionControl::wrapperFunctions(os, "env", "motionControl:", indent);
        LuaRobotAlgorithm::wrapperFunctions(os, "env",
                                            "robotAlgorithm:", indent);
        LuaForceControl::wrapperFunctions(os, "env", "forceControl:", indent);
        LuaRobotManage::wrapperFunctions(os, "env", "robotManage:", indent);
        LuaRobotState::wrapperFunctions(os, "env", "robotState:", indent);
        LuaSyncMove::wrapperFunctions(os, "env", "syncMove:", indent);
        LuaTrace::wrapperFunctions(os, "env", "trace:", indent);

        return os;
    }

private:
    RobotInterfacePtr self;
};
using LuaRobotInterfacePtr = std::shared_ptr<LuaRobotInterface>;
} // namespace aubo_script
} // namespace arcs
#endif
