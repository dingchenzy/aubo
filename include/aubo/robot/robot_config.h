#ifndef AUBO_SDK_ROBOT_CONFIG_H
#define AUBO_SDK_ROBOT_CONFIG_H

#include <vector>
#include <unordered_map>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT RobotConfig
{
public:
    RobotConfig();
    virtual ~RobotConfig();

    /**
     * 获取机器人的名字
     *
     * @return 返回机器人的名字
     *
     * @code Python函数原型
     * getName(self: pyaubo_sdk.RobotConfig) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getName() -> string
     * @endcode
     */
    std::string getName();

    /**
     * 获取机器人的自由度(从硬件抽象层读取)
     *
     * @return 返回机器人的自由度
     *
     * @code Python函数原型
     * getDof(self: pyaubo_sdk.RobotConfig) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getDof() -> number
     * @endcode
     */
    int getDof();

    /**
     * 获取机器人的伺服控制周期(从硬件抽象层读取)
     *
     * @return 机器人的伺服控制周期
     *
     * @code Python函数原型
     * getCycletime(self: pyaubo_sdk.RobotConfig) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getCycletime() -> number
     * @endcode
     */
    double getCycletime();

    /**
     * 获取默认的工具端加速度，单位m/s^2
     *
     * @return 默认的工具端加速度
     *
     * @code Python函数原型
     * getDefaultToolAcc(self: pyaubo_sdk.RobotConfig) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getDefaultToolAcc() -> number
     * @endcode
     */
    double getDefaultToolAcc();

    /**
     * 获取默认的工具端速度，单位m/s
     *
     * @return 默认的工具端速度
     *
     * @code Python函数原型
     * getDefaultToolSpeed(self: pyaubo_sdk.RobotConfig) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getDefaultToolSpeed() -> number
     * @endcode
     */
    double getDefaultToolSpeed();

    /**
     * 获取默认的关节加速度，单位rad/s^2
     *
     * @return 默认的关节加速度
     *
     * @code Python函数原型
     * getDefaultJointAcc(self: pyaubo_sdk.RobotConfig) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getDefaultJointAcc() -> number
     * @endcode
     */
    double getDefaultJointAcc();

    /**
     * 获取默认的关节速度，单位rad/s
     *
     * @return 默认的关节速度
     *
     * @code Python函数原型
     * getDefaultJointSpeed(self: pyaubo_sdk.RobotConfig) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getDefaultJointSpeed() -> number
     * @endcode
     */
    double getDefaultJointSpeed();

    /**
     * 获取机器人类型代码
     *
     * @return 机器人类型代码
     *
     * @code Python函数原型
     * getRobotType(self: pyaubo_sdk.RobotConfig) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getRobotType() -> string
     * @endcode
     */
    std::string getRobotType();

    /**
     * 获取机器人子类型代码
     *
     * @return 机器人子类型代码
     *
     * @code Python函数原型
     * getRobotSubType(self: pyaubo_sdk.RobotConfig) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getRobotSubType() -> string
     * @endcode
     */
    std::string getRobotSubType();

    /**
     * 获取控制柜类型代码
     *
     * @return 控制柜类型代码
     *
     * @code Python函数原型
     * getControlBoxType(self: pyaubo_sdk.RobotConfig) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getControlBoxType() -> number
     * @endcode
     */
    int getControlBoxType();

    /**
     * 设置安装位姿(机器人的基坐标系相对于世界坐标系)  world->base
     * 一般在多机器人系统中使用，默认为 [0,0,0,0,0,0]
     *
     * @param pose 安装位姿
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setMountingPose(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setMountingPose(pose: table) -> nil
     * @endcode
     */
    int setMountingPose(const std::vector<double> &pose);

    /**
     * 获取安装位姿(机器人的基坐标系相对于世界坐标系)
     *
     * @return 安装位姿
     *
     * @code Python函数原型
     * getMountingPose(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getMountingPose() -> table
     * @endcode
     */
    std::vector<double> getMountingPose();

    /**
     * 设置碰撞灵敏度等级
     * 数值越大越灵敏
     *
     * @param level 碰撞灵敏度等级
     * 0: 关闭碰撞检测功能
     * 1~9: 碰撞灵敏等级
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setCollisionLevel(self: pyaubo_sdk.RobotConfig, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setCollisionLevel(level: number) -> nil
     * @endcode
     */
    int setCollisionLevel(int level);

    /**
     * 获取碰撞灵敏度等级
     *
     * @return 碰撞灵敏度等级
     *
     * @code Python函数原型
     * getCollisionLevel(self: pyaubo_sdk.RobotConfig) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getCollisionLevel() -> number
     * @endcode
     */
    int getCollisionLevel();

    /**
     * 设置碰撞停止类型
     *
     * @param type 类型
     * 0: 碰撞停机
     * 1: 碰撞之后进入拖动模式
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setCollisionStopType(self: pyaubo_sdk.RobotConfig, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setCollisionStopType(type: number) -> nil
     * @endcode
     */
    int setCollisionStopType(int type);

    /**
     * 获取碰撞停止类型
     *
     * @return 返回碰撞停止类型
     *
     * @code Python函数原型
     * getCollisionStopType(self: pyaubo_sdk.RobotConfig) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getCollisionStopType() -> number
     * @endcode
     */
    int getCollisionStopType();

    /**
     * 设置机器人的 Home 位置
     *
     * @param positions 关节角度
     * @return
     */
    int setHomePosition(const std::vector<double> &positions);

    /**
     * 获取机器人 Home 位置
     *
     * @return
     */
    std::vector<double> getHomePosition();

    /**
     * 设置拖动阻尼
     *
     * @param damp 阻尼
     * @return 成功0，失败-1
     *
     * @code Python函数原型
     * setFreedriveDamp(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setFreedriveDamp(damp: table) -> number
     * @endcode
     */
    int setFreedriveDamp(const std::vector<double> &damp);

    /**
     * 获取拖动阻尼
     *
     * @return 拖动阻尼
     *
     * @code Python函数原型
     * getFreedriveDamp(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getFreedriveDamp() -> table
     * @endcode
     */
    std::vector<double> getFreedriveDamp();

    /**
     * 获取机器人DH参数
     * alpha a d theta beta
     *
     * @param real 读取真实参数(理论值+补偿值)或者理论参数
     * @return 返回机器人DH参数
     *
     * @code Python函数原型
     * getKinematicsParam(self: pyaubo_sdk.RobotConfig, arg0: bool) -> Dict[str,
     * List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * getKinematicsParam(real: boolean) -> table
     * @endcode
     */
    std::unordered_map<std::string, std::vector<double>> getKinematicsParam(
        bool real);

    /**
     * 获取指定温度下的DH参数补偿值
     * alpha a d theta beta
     *
     * @param ref_temperature 参考温度 ℃，默认20℃
     * @return 返回DH参数补偿值
     *
     * @code Python函数原型
     * getKinematicsCompensate(self: pyaubo_sdk.RobotConfig, arg0: float) ->
     * Dict[str, List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * getKinematicsCompensate(ref_temperature: number) -> table
     * @endcode
     */
    std::unordered_map<std::string, std::vector<double>>
    getKinematicsCompensate(double ref_temperature);

    /**
     * 设置运动学模型补偿参数
     * da dalpha dd dtheta dbeta
     *
     * @param param 补偿数据
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setPersistentParameters(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setPersistentParameters(param: string) -> nil
     * @endcode
     */
    int setPersistentParameters(const std::string &param);

    /**
     * 设置机器人关节零位
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setRobotZero(self: pyaubo_sdk.RobotConfig) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setRobotZero() -> nil
     * @endcode
     */
    int setRobotZero();

    /**
     * 获取可用的末端力矩传感器的名字
     *
     * @return 返回可用的末端力矩传感器的名字
     *
     * @code Python函数原型
     * getTcpForceSensorNames(self: pyaubo_sdk.RobotConfig) -> List[str]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpForceSensorNames() -> table
     * @endcode
     */
    std::vector<std::string> getTcpForceSensorNames();

    /**
     * 设置末端力矩传感器
     * 如果存在内置的末端力矩传感器，默认将使用内置的力矩传感器
     *
     * @param name 末端力矩传感器的名字
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * selectTcpForceSensor(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * selectTcpForceSensor(name: string) -> nil
     * @endcode
     */
    int selectTcpForceSensor(const std::string &name);

    /**
     * 设置传感器安装位姿
     *
     * @param sensor_pose 传感器安装位姿
     */
    int setTcpForceSensorPose(const std::vector<double> &sensor_pose);

    /**
     * 获取传感器安装位姿
     * @return 传感器安装位姿
     */
    std::vector<double> getTcpForceSensorPose();
    /**
     * 是否安装了末端力矩传感器
     *
     * @return 安装返回true; 没有安装返回false
     *
     * @code Python函数原型
     * hasTcpForceSensor(self: pyaubo_sdk.RobotConfig) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * hasTcpForceSensor() -> boolean
     * @endcode
     */
    bool hasTcpForceSensor();

    /**
     * 设置末端力矩偏移
     *
     * @param force_offset 末端力矩偏移
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setTcpForceOffset(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setTcpForceOffset(force_offset: table) -> nil
     * @endcode
     */
    int setTcpForceOffset(const std::vector<double> &force_offset);

    /**
     * 获取末端力矩偏移
     *
     * @return 返回末端力矩偏移
     *
     * @code Python函数原型
     * getTcpForceOffset(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpForceOffset() -> table
     * @endcode
     */
    std::vector<double> getTcpForceOffset();

    /**
     * 获取可用的底座力矩传感器的名字
     *
     * @return 返回可用的底座力矩传感器的名字
     *
     * @code Python函数原型
     * getBaseForceSensorNames(self: pyaubo_sdk.RobotConfig) -> List[str]
     * @endcode
     *
     * @code Lua函数原型
     * getBaseForceSensorNames() -> table
     * @endcode
     */
    std::vector<std::string> getBaseForceSensorNames();

    /**
     * 设置底座力矩传感器
     * 如果存在内置的底座力矩传感器，默认将使用内置的力矩传感器
     *
     * @param name 底座力矩传感器的名字
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * selectBaseForceSensor(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * selectBaseForceSensor(name: string) -> nil
     * @endcode
     */
    int selectBaseForceSensor(const std::string &name);

    /**
     * 是否安装了底座力矩传感器
     *
     * @return 安装返回true;没有安装返回false
     *
     * @code Python函数原型
     * hasBaseForceSensor(self: pyaubo_sdk.RobotConfig) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * hasBaseForceSensor() -> boolean
     * @endcode
     */
    bool hasBaseForceSensor();

    /**
     * 设置底座力矩偏移
     *
     * @param force_offset 底座力矩偏移
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setBaseForceOffset(self: pyaubo_sdk.RobotConfig, arg0: List[float]) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * setBaseForceOffset(force_offset: table) -> nil
     * @endcode
     */
    int setBaseForceOffset(const std::vector<double> &force_offset);

    /**
     * 获取底座力矩偏移
     *
     * @return 返回底座力矩偏移
     *
     * @code Python函数原型
     * getBaseForceOffset(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getBaseForceOffset() -> table
     * @endcode
     */
    std::vector<double> getBaseForceOffset();

    /**
     * 获取安全参数校验码 CRC32
     *
     * @return 返回安全参数校验码
     *
     * @code Python函数原型
     * getSafetyParametersCheckSum(self: pyaubo_sdk.RobotConfig) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getSafetyParametersCheckSum() -> number
     * @endcode
     */
    uint32_t getSafetyParametersCheckSum();

    /**
     * 发起确认安全配置参数请求:
     * 将安全配置参数写入到安全接口板flash或文件
     *
     * @param parameters 安全配置参数
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * confirmSafetyParameters(self: pyaubo_sdk.RobotConfig, arg0:
     * arcs::common_interface::RobotSafetyParameterRange) -> int
     * @endcode
     *
     * @code Lua函数原型
     *
     * @endcode
     */
    int confirmSafetyParameters(const RobotSafetyParameterRange &parameters);

    /**
     * 获取关节最大位置（物理极限）
     *
     * @return 返回关节最大位置
     *
     * @code Python函数原型
     * getJointMaxPositions(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointMaxPositions() -> table
     * @endcode
     */
    std::vector<double> getJointMaxPositions();

    /**
     * 获取关节最小位置（物理极限）
     *
     * @return 返回关节最小位置
     *
     * @code Python函数原型
     * getJointMinPositions(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointMinPositions() -> table
     * @endcode
     */
    std::vector<double> getJointMinPositions();

    /**
     * 获取关节最大速度（物理极限）
     *
     * @return 返回关节最大速度
     *
     * @code Python函数原型
     * getJointMaxSpeeds(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointMaxSpeeds() -> table
     * @endcode
     */
    std::vector<double> getJointMaxSpeeds();

    /**
     * 获取关节最大加速度（物理极限）
     *
     * @return 返回关节最大加速度
     *
     * @code Python函数原型
     * getJointMaxAccelerations(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getJointMaxAccelerations() -> table
     * @endcode
     */
    std::vector<double> getJointMaxAccelerations();

    /**
     * 获取TCP最大速度（物理极限）
     *
     * @return 返回TCP最大速度
     *
     * @code Python函数原型
     * getTcpMaxSpeeds(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpMaxSpeeds() -> table
     * @endcode
     */
    std::vector<double> getTcpMaxSpeeds();

    /**
     * 获取TCP最大加速度（物理极限）
     *
     * @return 返回TCP最大加速度
     *
     * @code Python函数原型
     * getTcpMaxAccelerations(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpMaxAccelerations() -> table
     * @endcode
     */
    std::vector<double> getTcpMaxAccelerations();

    /**
     * 设置机器人安装姿态
     *
     * @param gravity 安装姿态
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setGravity(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setGravity(gravity: table) -> nil
     * @endcode
     */
    int setGravity(const std::vector<double> &gravity);

    /**
     * 获取机器人的安装姿态
     * 如果机器人底座安装了姿态传感器，则从传感器读取数据，否则按照用户设置
     *
     * @return 返回安装姿态
     *
     * @code Python函数原型
     * getGravity(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getGravity() -> table
     * @endcode
     */
    std::vector<double> getGravity();

    /**
     * 设置TCP偏移
     *
     * @param offset TCP偏移
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setTcpOffset(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setTcpOffset(offset: table) -> nil
     * @endcode
     */
    int setTcpOffset(const std::vector<double> &offset);

    /**
     * 获取TCP偏移
     *
     * @return 返回TCP偏移
     *
     * @code Python函数原型
     * getTcpOffset(self: pyaubo_sdk.RobotConfig) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getTcpOffset() -> table
     * @endcode
     */
    std::vector<double> getTcpOffset();

    /**
     * 设置工具端质量、质心及惯量
     *
     * @param m 工具端质量
     * @param com 质心
     * @param inertial 惯量
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setToolInertial(self: pyaubo_sdk.RobotConfig, arg0: float, arg1:
     * List[float], arg2: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setToolInertial(m: number, com: table, inertial: table) -> nil
     * @endcode
     */
    int setToolInertial(double m, const std::vector<double> &com,
                        const std::vector<double> &inertial);

    /**
     * 设置末端负载
     *
     * @param m 质量
     * @param cog 重心
     * @param aom
     * @param inertia 惯量
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setPayload(self: pyaubo_sdk.RobotConfig, arg0: float, arg1: List[float],
     * arg2: List[float], arg3: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setPayload(m: number, cog: table, aom: table, inertia: table) -> nil
     * @endcode
     */
    int setPayload(double m, const std::vector<double> &cog,
                   const std::vector<double> &aom,
                   const std::vector<double> &inertia);

    /**
     * 获取末端负载
     *
     * @return 返回末端负载
     *
     * @code Python函数原型
     * getPayload(self: pyaubo_sdk.RobotConfig) -> Tuple[float, List[float],
     * List[float], List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * getPayload() -> number
     * @endcode
     */
    Payload getPayload();

    /**
     * 末端位姿是否在安全范围之内
     *
     * @param pose 末端位姿
     * @return 在安全范围内返回true; 反之返回false
     *
     * @code Python函数原型
     * toolSpaceInRange(self: pyaubo_sdk.RobotConfig, arg0: List[float]) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * toolSpaceInRange(pose: table) -> boolean
     * @endcode
     */
    bool toolSpaceInRange(const std::vector<double> &pose);

    /**
     * 发起固件升级请求，控制器软件将进入固件升级模式
     *
     * @param fw 固件路径
     *     pm://param/model/xx.bin
     *     /absolute/path/to/xx.bin
     *     relative/path/to/xx.bin
     * @return 成功返回0; 失败返回错误码
     * -AUBO_BAD_STATE: 当前运行时状态不处于 Stopped, 固件升级请求被拒绝
     *
     * @code Python函数原型
     * firmwareUpdate(self: pyaubo_sdk.RobotConfig, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * firmwareUpdate(fw: string) -> nil
     * @endcode
     */
    int firmwareUpdate(const std::string &fw);

    /**
     * 获取固件升级的进程
     *
     * @return 返回升级进程
     * std::string 代表步骤名称,
     * double 代表进度(0~1)，完成之后，返回("", 1)
     *
     * @code Python函数原型
     * getFirmwareUpdateProcess(self: pyaubo_sdk.RobotConfig) -> Tuple[str,
     * float]
     * @endcode
     *
     * @code Lua函数原型
     * getFirmwareUpdateProcess() -> table
     * @endcode
     */
    std::tuple<std::string, double> getFirmwareUpdateProcess();

protected:
    void *d_;
};
using RobotConfigPtr = std::shared_ptr<RobotConfig>;

// clang-format off
#define RobotConfig_DECLARES                                                     \
    _FUNC(RobotConfig, 0, getDof)                                                \
    _FUNC(RobotConfig, 0, getName)                                               \
    _FUNC(RobotConfig, 0, getCycletime)                                          \
    _FUNC(RobotConfig, 0, getRobotType)                                          \
    _FUNC(RobotConfig, 0, getRobotSubType)                                       \
    _FUNC(RobotConfig, 0, getControlBoxType)                                     \
    _FUNC(RobotConfig, 0, getDefaultToolAcc)                                     \
    _FUNC(RobotConfig, 0, getDefaultToolSpeed)                                   \
    _FUNC(RobotConfig, 0, getDefaultJointAcc)                                    \
    _FUNC(RobotConfig, 0, getDefaultJointSpeed)                                  \
    _INST(RobotConfig, 1, setMountingPose, pose)                                 \
    _FUNC(RobotConfig, 0, getMountingPose)                                       \
    _INST(RobotConfig, 1, setCollisionLevel, level);                             \
    _FUNC(RobotConfig, 0, getCollisionLevel);                                    \
    _INST(RobotConfig, 1, setCollisionStopType, type);                           \
    _FUNC(RobotConfig, 0, getCollisionStopType);                                 \
    _FUNC(RobotConfig, 1, setHomePosition, positions);                           \
    _FUNC(RobotConfig, 0, getHomePosition);                                      \
    _INST(RobotConfig, 1, setFreedriveDamp, damp);                               \
    _FUNC(RobotConfig, 0, getFreedriveDamp);                                     \
    _FUNC(RobotConfig, 0, getTcpForceSensorNames)                                \
    _INST(RobotConfig, 1, selectTcpForceSensor, name)                            \
    _FUNC(RobotConfig, 0, hasTcpForceSensor)                                     \
    _INST(RobotConfig, 1, setTcpForceOffset, force_offset)                       \
    _FUNC(RobotConfig, 0, getTcpForceOffset)                                     \
    _FUNC(RobotConfig, 0, getBaseForceSensorNames)                               \
    _INST(RobotConfig, 1, selectBaseForceSensor, name)                           \
    _FUNC(RobotConfig, 0, hasBaseForceSensor)                                    \
    _INST(RobotConfig, 1, setBaseForceOffset, force_offset)                      \
    _FUNC(RobotConfig, 0, getBaseForceOffset)                                    \
    _FUNC(RobotConfig, 1, setPersistentParameters, param)                        \
    _INST(RobotConfig, 0, setRobotZero)                                          \
    _FUNC(RobotConfig, 1, getKinematicsParam, real)                              \
    _FUNC(RobotConfig, 1, getKinematicsCompensate, ref_temperature)              \
    _FUNC(RobotConfig, 0, getSafetyParametersCheckSum)                           \
    _FUNC(RobotConfig, 1, confirmSafetyParameters, parameters)                   \
    _FUNC(RobotConfig, 0, getJointMaxPositions)                                  \
    _FUNC(RobotConfig, 0, getJointMinPositions)                                  \
    _FUNC(RobotConfig, 0, getJointMaxSpeeds)                                     \
    _FUNC(RobotConfig, 0, getJointMaxAccelerations)                              \
    _FUNC(RobotConfig, 0, getTcpMaxSpeeds)                                       \
    _FUNC(RobotConfig, 0, getTcpMaxAccelerations)                                \
    _FUNC(RobotConfig, 1, toolSpaceInRange, pose)                                \
    _INST(RobotConfig, 4, setPayload, m, cog, aom, inertia)                      \
    _FUNC(RobotConfig, 0, getPayload)                                            \
    _FUNC(RobotConfig, 0, getTcpOffset)                                          \
    _FUNC(RobotConfig, 0, getGravity)                                            \
    _INST(RobotConfig, 1, setGravity, gravity)                                   \
    _INST(RobotConfig, 1, setTcpOffset, offset)                                  \
    _INST(RobotConfig, 3, setToolInertial, m, com, inertial)                     \
    _FUNC(RobotConfig, 1, firmwareUpdate, fw)                                    \
    _FUNC(RobotConfig, 0, getFirmwareUpdateProcess)                              \
    _FUNC(RobotConfig, 1, setTcpForceSensorPose, sensor_pose)                    \
    _FUNC(RobotConfig, 0, getTcpForceSensorPose)

// clang-format on
} // namespace common_interface
} // namespace arcs
#endif // AUBO_SDK_ROBOT_CONFIG_H
