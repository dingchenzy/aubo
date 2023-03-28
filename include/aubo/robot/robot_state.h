#ifndef AUBO_SDK_ROBOT_STATE_INTERFACE_H
#define AUBO_SDK_ROBOT_STATE_INTERFACE_H

#include <vector>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT RobotState
{
public:
    RobotState();
    virtual ~RobotState();

    /**
     * 获取机器人的模式状态
     *
     * @return 机器人的模式状态
     *
     * @code Python函数原型
     * getRobotModeType(self: pyaubo_sdk.RobotState) ->
     * arcs::common_interface::RobotModeType
     * @endcode
     *
     * @code Lua函数原型
     * getRobotModeType() -> number
     * @endcode
     */
    RobotModeType getRobotModeType();

    /**
     * 获取安全模式
     *
     * @return 安全模式
     *
     * @code Python函数原型
     * getSafetyModeType(self: pyaubo_sdk.RobotState) ->
     * arcs::common_interface::SafetyModeType
     * @endcode
     *
     * @code Lua函数原型
     * getSafetyModeType() -> number
     * @endcode
     */
    SafetyModeType getSafetyModeType();

    /**
     * 获取机器人通电状态
     *
     * @return
     */
    bool isPowerOn();

    /**
     * 机器人是否已经停止下来
     *
     * @return 停止返回true; 反之返回false
     *
     * @code Python函数原型
     * isSteady(self: pyaubo_sdk.RobotState) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * isSteady() -> boolean
     * @endcode
     */
    bool isSteady();

    /**
     * 机器人是否发生了碰撞
     *
     * @return
     */
    bool isCollisionOccurred();

    /**
     * 机器人是否已经在安全限制之内
     *
     * @return 在安全限制之内返回true; 反之返回false
     *
     * @code Python函数原型
     * isWithinSafetyLimits(self: pyaubo_sdk.RobotState) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * isWithinSafetyLimits() -> boolean
     * @endcode
     */
    bool isWithinSafetyLimits();

    /**
     * 获取TCP的位姿
     *
     * @return TCP的位姿
     *
     * @code Python函数原型
     * getTcpPose(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpPose() -> table
     * @endcode
     */
    std::vector<double> getTcpPose();

    /**
     * 获取当前目标位姿
     *
     * @return 当前目标位姿
     *
     * @code Python函数原型
     * getTargetTcpPose(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTargetTcpPose() -> table
     * @endcode
     */
    std::vector<double> getTargetTcpPose();

    /**
     * 获取工具端的位姿（不带TCP偏移）
     *
     * @return 工具端的位姿
     *
     * @code Python函数原型
     * getToolPose(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getToolPose() -> table
     * @endcode
     */
    std::vector<double> getToolPose();

    /**
     * 获取TCP速度
     *
     * @return TCP速度
     *
     * @code Python函数原型
     * getTcpSpeed(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpSpeed() -> table
     * @endcode
     */
    std::vector<double> getTcpSpeed();

    /**
     * 获取TCP的力/力矩
     *
     * @return TCP的力/力矩
     *
     * @code Python函数原型
     * getTcpForce(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpForce() -> table
     * @endcode
     */
    std::vector<double> getTcpForce();

    /**
     * 获取肘部的位置
     *
     * @return 肘部的位置
     *
     * @code Python函数原型
     * getElbowPosistion(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getElbowPosistion() -> table
     * @endcode
     */
    std::vector<double> getElbowPosistion();

    /**
     * 获取肘部速度
     *
     * @return 肘部速度
     *
     * @code Python函数原型
     * getElbowVelocity(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getElbowVelocity() -> table
     * @endcode
     */
    std::vector<double> getElbowVelocity();

    /**
     * 获取基座力/力矩
     *
     * @return 基座力/力矩
     *
     * @code Python函数原型
     * getBaseForce(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getBaseForce() -> table
     * @endcode
     */
    std::vector<double> getBaseForce();

    /**
     * 获取TCP目标位姿
     *
     * @return TCP目标位姿
     *
     * @code Python函数原型
     * getTcpTargetPose(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpTargetPose() -> table
     * @endcode
     */
    std::vector<double> getTcpTargetPose();

    /**
     * 获取TCP目标速度
     *
     * @return TCP目标速度
     *
     * @code Python函数原型
     * getTcpTargetSpeed(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpTargetSpeed() -> table
     * @endcode
     */
    std::vector<double> getTcpTargetSpeed();

    /**
     * 获取TCP目标力/力矩
     *
     * @return TCP目标力/力矩
     *
     * @code Python函数原型
     * getTcpTargetForce(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpTargetForce() -> table
     * @endcode
     */
    std::vector<double> getTcpTargetForce();

    /**
     * 获取机械臂关节标志
     *
     * @return 机械臂关节标志
     *
     * @code Python函数原型
     * getJointState(self: pyaubo_sdk.RobotState) ->
     * List[arcs::common_interface::JointStateType]
     * @endcode
     *
     * @code Lua函数原型
     * getJointState() -> table
     * @endcode
     */
    std::vector<JointStateType> getJointState();

    /**
     * 获取关节的伺服状态
     *
     * @return 关节的伺服状态
     *
     * @code Python函数原型
     * getJointServoMode(self: pyaubo_sdk.RobotState) ->
     * List[arcs::common_interface::JointServoModeType]
     * @endcode
     *
     * @code Lua函数原型
     * getJointServoMode() -> table
     * @endcode
     */
    std::vector<JointServoModeType> getJointServoMode();

    /**
     * 获取机械臂关节角度
     *
     * @return 机械臂关节角度
     *
     * @code Python函数原型
     * getJointPositions(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointPositions() -> table
     * @endcode
     */
    std::vector<double> getJointPositions();

    /**
     * 获取机械臂关节速度
     *
     * @return 机械臂关节速度
     *
     * @code Python函数原型
     * getJointSpeeds(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointSpeeds() -> table
     * @endcode
     */
    std::vector<double> getJointSpeeds();

    /**
     * 获取机械臂关节加速度
     *
     * @return 机械臂关节加速度
     *
     * @code Python函数原型
     * getJointAccelerations(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointAccelerations() -> table
     * @endcode
     */
    std::vector<double> getJointAccelerations();

    /**
     * 获取机械臂关节力矩
     *
     * @return 机械臂关节力矩
     *
     * @code Python函数原型
     * getJointTorqueSensors(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointTorqueSensors() -> table
     * @endcode
     */
    std::vector<double> getJointTorqueSensors();

    /**
     * 获取底座力传感器读数
     *
     * @return 底座力传感器读数
     *
     * @code Python函数原型
     * getBaseForceSensor(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getBaseForceSensor() -> table
     * @endcode
     */
    std::vector<double> getBaseForceSensor();

    /**
     * 获取TCP力传感器读数
     *
     * @return TCP力传感器读数
     *
     * @code Python函数原型
     * getTcpForceSensors(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpForceSensors() -> table
     * @endcode
     */
    std::vector<double> getTcpForceSensors();

    /**
     * 获取机械臂关节电流
     *
     * @return 机械臂关节电流
     *
     * @code Python函数原型
     * getJointCurrents(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointCurrents() -> table
     * @endcode
     */
    std::vector<double> getJointCurrents();

    /**
     * 获取机械臂关节电压
     *
     * @return 机械臂关节电压
     *
     * @code Python函数原型
     * getJointVoltages(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointVoltages() -> table
     * @endcode
     */
    std::vector<double> getJointVoltages();

    /**
     * 获取机械臂关节温度
     *
     * @return 机械臂关节温度
     *
     * @code Python函数原型
     * getJointTemperatures(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointTemperatures() -> table
     * @endcode
     */
    std::vector<double> getJointTemperatures();

    /**
     * 获取关节全球唯一ID
     *
     * @return 关节全球唯一ID
     *
     * @code Python函数原型
     * getJointUniqueIds(self: pyaubo_sdk.RobotState) -> List[str]
     * @endcode
     *
     * @code Lua函数原型
     * getJointUniqueIds() -> table
     * @endcode
     */
    std::vector<std::string> getJointUniqueIds();

    /**
     * 获取关节固件版本
     *
     * @return 关节固件版本
     *
     * @code Python函数原型
     * getJointFirmwareVersions(self: pyaubo_sdk.RobotState) -> List[int]
     * @endcode
     *
     * @code Lua函数原型
     * getJointFirmwareVersions() -> table
     * @endcode
     */
    std::vector<int> getJointFirmwareVersions();

    /**
     * 获取关节硬件版本
     *
     * @return 关节硬件版本
     *
     * @code Python函数原型
     * getJointHardwareVersions(self: pyaubo_sdk.RobotState) -> List[int]
     * @endcode
     *
     * @code Lua函数原型
     * getJointHardwareVersions() -> table
     * @endcode
     */
    std::vector<int> getJointHardwareVersions();

    /**
     * 获取MasterBoard全球唯一ID
     *
     * @return MasterBoard全球唯一ID
     *
     * @code Python函数原型
     * getMasterBoardUniqueId(self: pyaubo_sdk.RobotState) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getMasterBoardUniqueId() -> string
     * @endcode
     */
    std::string getMasterBoardUniqueId();

    /**
     * 获取MasterBoard固件版本
     *
     * @return MasterBoard固件版本
     *
     * @code Python函数原型
     * getMasterBoardFirmwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getMasterBoardFirmwareVersion() -> number
     * @endcode
     */
    int getMasterBoardFirmwareVersion();

    /**
     * 获取MasterBoard硬件版本
     *
     * @return MasterBoard硬件版本
     *
     * @code Python函数原型
     * getMasterBoardHardwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getMasterBoardHardwareVersion() -> number
     * @endcode
     */
    int getMasterBoardHardwareVersion();

    /**
     * 获取SlaveBoard全球唯一ID
     *
     * @return SlaveBoard全球唯一ID
     *
     * @code Python函数原型
     * getSlaveBoardUniqueId(self: pyaubo_sdk.RobotState) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getSlaveBoardUniqueId() -> string
     * @endcode
     */
    std::string getSlaveBoardUniqueId();

    /**
     * 获取SlaveBoard固件版本
     *
     * @return SlaveBoard固件版本
     *
     * @code Python函数原型
     * getSlaveBoardFirmwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getSlaveBoardFirmwareVersion() -> number
     * @endcode
     */
    int getSlaveBoardFirmwareVersion();

    /**
     * 获取SlaveBoard硬件版本
     *
     * @return SlaveBoard硬件版本
     *
     * @code Python函数原型
     * getSlaveBoardHardwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getSlaveBoardHardwareVersion() -> number
     * @endcode
     */
    int getSlaveBoardHardwareVersion();

    /**
     * 获取工具端全球唯一ID
     *
     * @return 工具端全球唯一ID
     *
     * @code Python函数原型
     * getToolUniqueId(self: pyaubo_sdk.RobotState) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getToolUniqueId() -> string
     * @endcode
     */
    std::string getToolUniqueId();

    /**
     * 获取工具端固件版本
     *
     * @return 工具端固件版本
     *
     * @code Python函数原型
     * getToolFirmwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolFirmwareVersion() -> number
     * @endcode
     */
    int getToolFirmwareVersion();

    /**
     * 获取工具端硬件版本
     *
     * @return 工具端硬件版本
     *
     * @code Python函数原型
     * getToolHardwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getToolHardwareVersion() -> number
     * @endcode
     */
    int getToolHardwareVersion();

    /**
     * 获取底座全球唯一ID
     *
     * @return 底座全球唯一ID
     *
     * @code Python函数原型
     * getPedestalUniqueId(self: pyaubo_sdk.RobotState) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getPedestalUniqueId() -> string
     * @endcode
     */
    std::string getPedestalUniqueId();

    /**
     * 获取底座固件版本
     *
     * @return 底座固件版本
     *
     * @code Python函数原型
     * getPedestalFirmwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getPedestalFirmwareVersion() -> number
     * @endcode
     */
    int getPedestalFirmwareVersion();

    /**
     * 获取底座硬件版本
     *
     * @return 底座硬件版本
     *
     * @code Python函数原型
     * getPedestalHardwareVersion(self: pyaubo_sdk.RobotState) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getPedestalHardwareVersion() -> number
     * @endcode
     */
    int getPedestalHardwareVersion();

    /**
     * 获取机械臂关节目标位置角度
     *
     * @return 机械臂关节目标位置角度
     *
     * @code Python函数原型
     * getJointTargetPositions(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointTargetPositions() -> table
     * @endcode
     */
    std::vector<double> getJointTargetPositions();

    /**
     * 获取机械臂关节目标速度
     *
     * @return 机械臂关节目标速度
     *
     * @code Python函数原型
     * getJointTargetSpeeds(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointTargetSpeeds() -> table
     * @endcode
     */
    std::vector<double> getJointTargetSpeeds();

    /**
     * 获取机械臂关节目标加速度
     *
     * @return 机械臂关节目标加速度
     *
     * @code Python函数原型
     * getJointTargetAccelerations(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointTargetAccelerations() -> table
     * @endcode
     */
    std::vector<double> getJointTargetAccelerations();

    /**
     * 获取机械臂关节目标力矩
     *
     * @return 机械臂关节目标力矩
     *
     * @code Python函数原型
     * getJointTargetTorques(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointTargetTorques() -> table
     * @endcode
     */
    std::vector<double> getJointTargetTorques();

    /**
     * 获取机械臂关节目标电流
     *
     * @return 机械臂关节目标电流
     *
     * @code Python函数原型
     * getJointTargetCurrents(self: pyaubo_sdk.RobotState) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointTargetCurrents() -> table
     * @endcode
     */
    std::vector<double> getJointTargetCurrents();

    /**
     * 获取控制柜类型，可能包含控制柜的详细信息
     *
     * @return
     */
    std::string getControlBoxType();

    /**
     * 获取控制柜温度
     *
     * @return 控制柜温度
     *
     * @code Python函数原型
     * getControlBoxTemperature(self: pyaubo_sdk.RobotState) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getControlBoxTemperature() -> number
     * @endcode
     */
    double getControlBoxTemperature();

    /**
     * 获取母线电压
     *
     * @return 母线电压
     *
     * @code Python函数原型
     * getMainVoltage(self: pyaubo_sdk.RobotState) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getMainVoltage() -> number
     * @endcode
     */
    double getMainVoltage();

    /**
     * 获取母线电流
     *
     * @return 母线电流
     *
     * @code Python函数原型
     * getMainCurrent(self: pyaubo_sdk.RobotState) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getMainCurrent() -> number
     * @endcode
     */
    double getMainCurrent();

    /**
     * 获取机器人电压
     *
     * @return 机器人电压
     *
     * @code Python函数原型
     * getRobotVoltage(self: pyaubo_sdk.RobotState) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getRobotVoltage() -> number
     * @endcode
     */
    double getRobotVoltage();

    /**
     * 获取机器人电流
     *
     * @return 机器人电流
     *
     * @code Python函数原型
     * getRobotCurrent(self: pyaubo_sdk.RobotState) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getRobotCurrent() -> number
     * @endcode
     */
    double getRobotCurrent();

protected:
    void *d_;
};
using RobotStatePtr = std::shared_ptr<RobotState>;

// clang-format off
#define RobotState_DECLARES                              \
    _FUNC(RobotState, 0, getRobotModeType)               \
    _FUNC(RobotState, 0, getSafetyModeType)              \
    _FUNC(RobotState, 0, isPowerOn)                      \
    _FUNC(RobotState, 0, isSteady)                       \
    _FUNC(RobotState, 0, isCollisionOccurred)            \
    _FUNC(RobotState, 0, isWithinSafetyLimits)           \
    _FUNC(RobotState, 0, getTcpPose)                     \
    _FUNC(RobotState, 0, getTargetTcpPose)               \
    _FUNC(RobotState, 0, getToolPose)                    \
    _FUNC(RobotState, 0, getTcpSpeed)                    \
    _FUNC(RobotState, 0, getTcpForce)                    \
    _FUNC(RobotState, 0, getElbowPosistion)              \
    _FUNC(RobotState, 0, getElbowVelocity)               \
    _FUNC(RobotState, 0, getBaseForce)                   \
    _FUNC(RobotState, 0, getTcpTargetPose)               \
    _FUNC(RobotState, 0, getTcpTargetSpeed)              \
    _FUNC(RobotState, 0, getTcpTargetForce)              \
    _FUNC(RobotState, 0, getJointState)                  \
    _FUNC(RobotState, 0, getJointServoMode)              \
    _FUNC(RobotState, 0, getJointPositions)              \
    _FUNC(RobotState, 0, getJointSpeeds)                 \
    _FUNC(RobotState, 0, getJointAccelerations)          \
    _FUNC(RobotState, 0, getJointTorqueSensors)          \
    _FUNC(RobotState, 0, getBaseForceSensor)             \
    _FUNC(RobotState, 0, getTcpForceSensors)             \
    _FUNC(RobotState, 0, getJointCurrents)               \
    _FUNC(RobotState, 0, getJointVoltages)               \
    _FUNC(RobotState, 0, getJointTemperatures)           \
    _FUNC(RobotState, 0, getJointUniqueIds)              \
    _FUNC(RobotState, 0, getJointFirmwareVersions)       \
    _FUNC(RobotState, 0, getJointHardwareVersions)       \
    _FUNC(RobotState, 0, getMasterBoardUniqueId)         \
    _FUNC(RobotState, 0, getMasterBoardFirmwareVersion)  \
    _FUNC(RobotState, 0, getMasterBoardHardwareVersion)  \
    _FUNC(RobotState, 0, getSlaveBoardUniqueId)          \
    _FUNC(RobotState, 0, getSlaveBoardFirmwareVersion)   \
    _FUNC(RobotState, 0, getSlaveBoardHardwareVersion)   \
    _FUNC(RobotState, 0, getToolUniqueId)                \
    _FUNC(RobotState, 0, getToolFirmwareVersion)         \
    _FUNC(RobotState, 0, getToolHardwareVersion)         \
    _FUNC(RobotState, 0, getPedestalUniqueId)            \
    _FUNC(RobotState, 0, getPedestalFirmwareVersion)     \
    _FUNC(RobotState, 0, getPedestalHardwareVersion)     \
    _FUNC(RobotState, 0, getJointTargetPositions)        \
    _FUNC(RobotState, 0, getJointTargetSpeeds)           \
    _FUNC(RobotState, 0, getJointTargetAccelerations)    \
    _FUNC(RobotState, 0, getJointTargetTorques)          \
    _FUNC(RobotState, 0, getJointTargetCurrents)         \
    _FUNC(RobotState, 0, getControlBoxType)              \
    _FUNC(RobotState, 0, getControlBoxTemperature)       \
    _FUNC(RobotState, 0, getMainVoltage)                 \
    _FUNC(RobotState, 0, getMainCurrent)                 \
    _FUNC(RobotState, 0, getRobotVoltage)                \
    _FUNC(RobotState, 0, getRobotCurrent)
// clang-format on
} // namespace common_interface
} // namespace arcs
#endif // AUBO_SDK_ROBOT_STATE_INTERFACE_H
