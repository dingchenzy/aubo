#ifndef AUBO_SCRIPT_BINDING_LUA_AUBO_CONTROL_API_H
#define AUBO_SCRIPT_BINDING_LUA_AUBO_CONTROL_API_H

#include "./robot_interface.h"
#include "./math.h"
#include "./socket.h"
#include "./serial.h"
#include "./system_info.h"
#include "./runtime_machine.h"
#include "./register_control.h"

namespace arcs {
namespace aubo_script {

class LuaAuboApi
{
public:
    LuaAuboApi(const AuboApiPtr &_impl) : self(_impl)
    {
        math_ = std::make_shared<LuaMath>(self->getMath());
        socket_ = std::make_shared<LuaSocket>(self->getSocket());
        serial_ = std::make_shared<LuaSerial>(self->getSerial());
        system_info_ = std::make_shared<LuaSystemInfo>(self->getSystemInfo());
        runtime_machine_ =
            std::make_shared<LuaRuntimeMachine>(self->getRuntimeMachine());
        register_control_ =
            std::make_shared<LuaRegisterControl>(self->getRegisterControl());
    }

    LuaMathPtr getMath() { return math_; }
    LuaSocketPtr getSocket() { return socket_; }
    LuaSerialPtr getSerial() { return serial_; }
    LuaSystemInfoPtr getSystemInfo() { return system_info_; }
    LuaRuntimeMachinePtr getRuntimeMachine() { return runtime_machine_; }
    LuaRegisterControlPtr getRegisterControl() { return register_control_; }

    sol::nested<std::vector<std::string>> getRobotNames()
    {
        return sol::as_nested(self->getRobotNames());
    }
    LuaRobotInterfacePtr getRobotInterface(const std::string &name)
    {
        return std::make_shared<LuaRobotInterface>(
            self->getRobotInterface(name));
    }

    static void registerLuaUserType(sol::state_view &L)
    {
#define ENUM_ITEM(c, n, ...) #c, T::c,

#define LUA_REGISTER_ENUM(ENUM)                                        \
    {                                                                  \
        using T = ENUM;                                                \
        L[#ENUM] = L.create_table_with(ENUM_##ENUM##_DECLARES "_", 0); \
    }

        // 注册枚举类型
        LUA_REGISTER_ENUM(AuboErrorCodes)
        LUA_REGISTER_ENUM(RuntimeState)
        LUA_REGISTER_ENUM(RobotModeType)
        LUA_REGISTER_ENUM(SafetyModeType)
        LUA_REGISTER_ENUM(OperationalModeType)
        LUA_REGISTER_ENUM(RobotControlModeType)
        LUA_REGISTER_ENUM(JointServoModeType)
        LUA_REGISTER_ENUM(JointStateType)
        LUA_REGISTER_ENUM(StandardInputAction)
        LUA_REGISTER_ENUM(StandardOutputRunState)
        LUA_REGISTER_ENUM(SafetyInputAction)
        LUA_REGISTER_ENUM(SafetyOutputRunState)
        LUA_REGISTER_ENUM(TaskFrameType)
        LUA_REGISTER_ENUM(TraceLevel)

        LuaMath::registerLuaUserType(L);
        LuaSerial::registerLuaUserType(L);
        LuaSocket::registerLuaUserType(L);
        LuaSystemInfo::registerLuaUserType(L);
        LuaRuntimeMachine::registerLuaUserType(L);
        LuaRegisterControl::registerLuaUserType(L);
        LuaRobotInterface::registerLuaUserType(L);

        sol::automagic_enrollments enrollments;
        enrollments.default_constructor = false; // 不生成构造函数
        enrollments.destructor = false;
        auto u = L.new_usertype<LuaAuboApi>("AuboApi", enrollments);

#define _INST(_, n, f, ...) u.set_function(#f, &LuaAuboApi::f);
#define _FUNC(_, n, f, ...) u.set_function(#f, &LuaAuboApi::f);
        AuboApi_DECLARES
#undef _INST
#undef _FUNC
    }

    static std::ostream &generateTestScript(std::ostream &os,
                                            const char *prefix = "AuboApi:")
    {
        auto p = prefix;
        LuaMath::generateTestScript(os, p);
        LuaSystemInfo::generateTestScript(os, p);
        LuaRuntimeMachine::generateTestScript(os, p);
        LuaRegisterControl::generateTestScript(os, p);
        LuaRobotInterface::generateTestScript(os, p);

        return os;
    }

    static std::ostream &wrapperFunctions(std::ostream &os)
    {
        os << "return function(api, n)" << std::endl;
        os << "  local env = {}" << std::endl;
        os << "  local math = api:getMath()" << std::endl;
        os << "  local socket = api:getSocket()" << std::endl;
        os << "  local serial = api:getSerial()" << std::endl;
        os << "  local systemInfo = api:getSystemInfo()" << std::endl;
        os << "  local runtimeMachine = api:getRuntimeMachine()" << std::endl;
        os << "  local registerControl = api:getRegisterControl()" << std::endl;

        os << "  local robot_name = assert(api:getRobotNames()[n])"
           << std::endl;
        os << "  local robot = assert(api:getRobotInterface(robot_name))"
           << std::endl;
        os << "  local forceControl = robot:getForceControl()" << std::endl;
        os << "  local ioControl = robot:getIoControl()" << std::endl;
        os << "  local motionControl = robot:getMotionControl()" << std::endl;
        os << "  local robotAlgorithm = robot:getRobotAlgorithm()" << std::endl;
        os << "  local robotConfig = robot:getRobotConfig()" << std::endl;
        os << "  local robotManage = robot:getRobotManage()" << std::endl;
        os << "  local robotState = robot:getRobotState()" << std::endl;
        os << "  local syncMove = robot:getSyncMove()" << std::endl;
        os << "  local trace = robot:getTrace()" << std::endl;

        LuaMath::wrapperFunctions(os, "env", "math:", 1);
        LuaSocket::wrapperFunctions(os, "env", "socket:", 1);
        LuaSerial::wrapperFunctions(os, "env", "serial:", 1);
        LuaSystemInfo::wrapperFunctions(os, "env", "systemInfo:", 1);
        LuaRuntimeMachine::wrapperFunctions(os, "env", "runtimeMachine:", 1);
        LuaRegisterControl::wrapperFunctions(os, "env", "registerControl:", 1);
        LuaRobotInterface::wrapperFunctions(os, 1);
        os << "  return env" << std::endl;
        os << "end" << std::endl;
        return os;
    }

private:
    AuboApiPtr self;
    LuaMathPtr math_;
    LuaSocketPtr socket_;
    LuaSerialPtr serial_;
    LuaSystemInfoPtr system_info_;
    LuaRuntimeMachinePtr runtime_machine_;
    LuaRegisterControlPtr register_control_;
};
using LuaAuboApiPtr = std::shared_ptr<LuaAuboApi>;
} // namespace aubo_script
} // namespace arcs
#endif
