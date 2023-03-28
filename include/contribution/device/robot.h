#ifndef COMMON_INTERFACE_ROBOTHW_H
#define COMMON_INTERFACE_ROBOTHW_H

#include <stdint.h>
#include <string.h>
#include <vector>
#include <array>
#include <memory>
#include <functional>
#include <map>
#include <unordered_map>
#include <string>
#include <optional>

#include <aubo/type_def.h>
#include <contribution/device/sensor.h>
#include <contribution/logging.h>

namespace arcs {
namespace common_interface {

/// 关节的伺服模式, 可以参考标准的伺服驱动器
/// https://www.cnblogs.com/21207-ihome/p/7722633.html
enum class JointServoMode : int
{
    None = -1,          ///<
    Open = 0,           ///<
    Current,            ///<
    ProfileVelocity,    ///<
    ProfilePosition,    ///< 位置模式
    CyclicSyncPosition, ///<
    CyclicSyncVelocity, ///<
    CyclicSyncTorque,   ///<
};

/// 关节状态
enum class JointMode : int
{
    Poweroff = 0, ///< 节点未连接到接口板或者已经断电
    Idle,         ///< 节点空闲
    Fault,        ///< 节点错误，节点停止伺服运动，刹车抱死
    Booting,      ///< 正在启动中
    Running,      ///< 节点伺服
    NoResponding, ///< 没有通讯无响应
    ShuttingDown, ///< 正在关闭
    Bootload,     ///< 节点bootloader状态，暂停一切通讯
};

/// 机械臂伺服运动模式枚举
enum class RobotServoMode : int
{
    None = (int)0, ///< 位置伺服控制模式
    Position,      ///< 位置伺服控制模式
    Velocity,      ///< 速度伺服控制模式
    Force,         ///< 力伺服控制模式
    Collision,     ///< 碰撞模式
};

/// 机械臂错误代码(错误标志位)结构体
struct RobotErrorCode
{
    RobotErrorCode() { memset(this, 0, sizeof(RobotErrorCode)); }
    RobotErrorCode(uint64_t val) { memcpy(this, &val, sizeof(RobotErrorCode)); }
    bool operator==(RobotErrorCode other)
    {
        return (memcmp(this, &other, sizeof(RobotErrorCode)) == 0);
    }
    RobotErrorCode operator=(uint64_t val)
    {
        memcpy(this, &val, sizeof(RobotErrorCode));
        return *this;
    }
    operator bool()
    {
        RobotErrorCode err;
        return (memcmp(this, &err, sizeof(RobotErrorCode)) != 0);
    }
    RobotErrorCode zero()
    {
        memset(this, 0, sizeof(RobotErrorCode));
        return *this;
    }
    uint64_t toInteger()
    {
        uint64_t val;
        memcpy(&val, this, sizeof(RobotErrorCode));
        return val;
    }

    uint64_t JointsFault : 1; ///< 关节故障，具体可以查看关节的错误代码
    uint64_t CommonError : 1;         ///< 关节错误
    uint64_t UnmatchRobotType : 1;    ///< 机械臂类型错误
    uint64_t AccSensorFault : 1;      ///< 加速度计芯片错误
    uint64_t EncoderLineError : 1;    ///< 编码器线数错误
    uint64_t EnterHandGuideError : 1; ///< 进入拖动示教模式错误
    uint64_t ExitHandGuideError : 1;  ///< 退出拖动示教模式错误
    uint64_t PacketCountDisagree : 1; ///< MAC数据中断错误
    uint64_t JointFWVersionError : 1; ///< 驱动器版本错误（关节固件版本不一致）
    uint64_t PayloadError : 1;    ///< 工具不存在
    uint64_t PwerOnTestError : 1; ///< 上电测试失败
    uint64_t DrvFwVersionLow : 1; ///< 驱动器固件版本太低
    uint64_t BufferOverflow : 1;  ///<

    uint64_t InterfaceBrdInitFailed : 1; ///< 初始化失败

    uint64_t SlaveBrdCommError : 1;     ///< 接口板MCU通讯失败
    uint64_t RS485CommError : 1;        ///< RS485通信失败(扩展板)
    uint64_t ProtectionStopTimeout : 1; ///< 接口板MCU通讯失败
    uint64_t ReduceModeTimeout : 1;     ///< 接口板MCU通讯失败

    uint64_t InterfaceBrdDisconnected : 1; ///< 上位机和接口板通讯错误
};

struct JointErrorCode
{
    JointErrorCode() { memset(this, 0, sizeof(JointErrorCode)); }
    JointErrorCode(uint64_t val) { memcpy(this, &val, sizeof(JointErrorCode)); }
    bool operator==(JointErrorCode other)
    {
        return (memcmp(this, &other, sizeof(JointErrorCode)) == 0);
    }
    JointErrorCode operator=(uint64_t val)
    {
        memcpy(this, &val, sizeof(JointErrorCode));
        return *this;
    }
    JointErrorCode operator||(JointErrorCode c)
    {
        JointErrorCode retval = *this;
#define JEOR(f) \
    if (c.f)    \
        retval.f = 1;

        JEOR(CommonError);
        JEOR(OverCurrent);
        JEOR(OverVoltage);
        JEOR(LowVoltage);
        JEOR(HighTemperature);
        JEOR(HallFault);
        JEOR(EncoderFault);
        JEOR(AbsEncoderFault);
        JEOR(CurrentCalibFault);
        JEOR(CurrentQFault);
        JEOR(EncoderPollution);
        JEOR(EncodeZSignal);
        JEOR(EncodeInvalid);
        JEOR(IMUFault);
        JEOR(TempSensorFault);
        JEOR(CANFault);
        JEOR(CurrentFault);
        JEOR(PosOutRange);
        JEOR(OverSpeed);
        JEOR(OverAcc);
        JEOR(FollowError);
        JEOR(TargetPosOutRange);
        JEOR(OverTargetSpeed);
        JEOR(Collision);
        return retval;
    }
    operator bool()
    {
        JointErrorCode err;
        return (memcmp(this, &err, sizeof(JointErrorCode)) != 0);
    }
    JointErrorCode zero()
    {
        memset(this, 0, sizeof(JointErrorCode));
        return *this;
    }
    uint64_t toInteger()
    {
        uint64_t val;
        memcpy(&val, this, sizeof(JointErrorCode));
        return val;
    }

    uint64_t CommonError : 1;       ///< 通用错误
    uint64_t OverCurrent : 1;       ///< 过流
    uint64_t OverVoltage : 1;       ///< 过压
    uint64_t LowVoltage : 1;        ///< 欠压
    uint64_t HighTemperature : 1;   ///< 过温
    uint64_t HallFault : 1;         ///< 霍尔错误
    uint64_t EncoderFault : 1;      ///< 编码器错误
    uint64_t AbsEncoderFault : 1;   ///< 绝对编码器错误
    uint64_t CurrentCalibFault : 1; ///< 电流检测错误
    uint64_t CurrentQFault : 1;     ///< 电流检测错误
    uint64_t EncoderPollution : 1;  ///< 编码器污染
    uint64_t EncodeZSignal : 1;     ///< 编码器Z信号错误
    uint64_t EncodeInvalid : 1;     ///< 编码器校准失效
    uint64_t IMUFault : 1;          ///< IMU传感器失效
    uint64_t TempSensorFault : 1;   ///< 温度传感器出错
    uint64_t CANFault : 1;          ///< CAN总线通讯错误
    uint64_t CurrentFault : 1;      ///< 当前电流错误
    uint64_t PosOutRange : 1;       ///< 当前位置错误
    uint64_t OverSpeed : 1;         ///< 关节超速错误
    uint64_t OverAcc : 1;           ///< 关节加速度过大错误
    uint64_t FollowError : 1;       ///< 跟踪精度错误
    uint64_t TargetPosOutRange : 1; ///< 目标位置超范围
    uint64_t OverTargetSpeed : 1;   ///< 目标速度超范围
    uint64_t OverTargetCurrent : 1; ///< 目标电流超范围
    uint64_t Collision : 1;         ///< 碰撞
    uint64_t ADCZeroOffset : 1;     ///< ADC 零点偏移错误
    uint64_t IpmNTC : 1;            ///< NTC故障
    uint64_t ShrotCircuit : 1;      ///< 短路故障
    uint64_t MotorStall : 1;        ///< 电机堵转
    uint64_t AbsMt : 1;             ///< 绝对编码器多圈故障
    uint64_t MotorPhaseLose : 1;    ///< 电机缺相
    uint64_t BrakeErr : 1;          ///< 刹车故障
    uint64_t ReducerOverTemp : 1;   ///< 减速机过温
    uint64_t ReducerNTC : 1;        ///< 减速机NTC故障
    uint64_t FirmwareUpdate : 1;    ///< 固件升级错误
    uint64_t FlashOP : 1;           ///< FLASH操作故障
    uint64_t ExtABSEnc : 1;         ///< 电机端编码器通信故障
};

struct SafetyMonitorStatus
{
    std::optional<bool> robot_emergency_stop;
    std::optional<bool> system_emergency_stop;
    std::optional<bool> reduced_mode;
    std::optional<bool> safeguard_stop;
    std::optional<bool> automatic_safeguard_stop;
    std::optional<bool> safeguard_reset;
    std::optional<bool> operational_mode;
    std::optional<bool> three_position_switch;
    std::optional<bool> freedrive;

    bool violation; ///< 是否安全超限
};

/// 联动IO的工程输出状态或者输入动作
enum LinkProgramState : int
{
    LPS_None = 0,
    LPS_Stop = 1,
    LPS_Start = 2,
    LPS_Pause = 3,
    LPS_Resume = 4,
};

/// 联动输入状态
struct LinkInputStatus
{
    uint16_t program_state : 3;     ///< 工程运行状态
    uint16_t goto_1st_position : 1; ///< 运动到 初始 位置
    uint16_t f1 : 1;
    uint16_t f2 : 1;
    uint16_t f3 : 1;
    uint16_t f4 : 1;
    uint16_t f5 : 1;
    uint16_t f6 : 1;
    uint16_t remote_on : 1;  ///< 远程开机
    uint16_t remote_off : 1; ///< 远程关机
};

/// 联动输出状态
struct LinkOutputStatus
{
    uint16_t program_state : 3;    ///< 工程运行状态
    uint16_t at_home_position : 1; ///< 是否在 Home 位置
};

class RobotHardware;
using RobotHardwarePtr = std::shared_ptr<RobotHardware>;

/**
 * 机器人硬件抽象层类, 设计原则如下:
 *  1、硬件抽象层对实际的机器人做了一层抽象，供控制器软件调用
 *  2、硬件抽象层提供包括配置、设置、读取等操作相关功能函数
 *  3、一些与机器人本体相关，但是不需要控制器软件设置的参数可以放在配置文件中
 *  4、接口设计应该保证向下兼容性(新版本的硬件抽象层能被旧版本的控制器软件使用)
 *
 * 如果注释中没有指定单位，则使用国际标准单位
 *  - 长度 m
 *  - 质量 kg
 *  - 关节位置 rad
 *  - 关节速度 rad/s
 *  - 关节加速度 rad/s^2
 *  - 电压 V
 *  - 电流 A
 *  - 温度 ℃
 */
class RobotHardware
{
public:
    enum ReturnValue : int
    {
        RET_OK = 0,            ///< 没有错误
        RET_NO_PERM = -1,      ///< Operation not permitted
        RET_TIMEOUT = -2,      ///< 操作超时
        RET_DISCONNECT = -3,   ///< 通讯断开连接
        RET_LOST_PACKAGE = -4, ///< 丢失关键数据包
        RET_JOINT_FAULT = -5,
        RET_NETWORK_HW_FAULT = -6,
    };

    virtual ~RobotHardware() = default;

    /**
     * 设置sleep函数
     *
     * @return
     */
    virtual void setSleepFunc(std::function<void(int ms)> &&sleep_func) = 0;

    /**
     * 获取sleep函数
     *
     * @return
     */
    virtual std::function<void(int)> getSleepFunc() = 0;

    /**
     * 获取控制柜型号
     *
     * @return
     */
    virtual std::string getControlBoxType() = 0;

    /**
     * 获取机器人自由度(关节数量)
     * @param 无
     * @return　关节数量 ARM_DOF
     */
    virtual int getJointNum() const = 0;

    /**
     * 与接口板建立连接
     * @param 无
     * @return　调用成功返回true; 错误返回false
     */
    virtual int connectToMasterBoard() = 0;

    /**
     * 是否与接口板建立了连接
     *
     * @return
     */
    virtual bool isConnected() const = 0;

    /**
     * 反初始化机器人（退出, 机械臂本体断电，断开与接口板的通讯连接）
     *
     * @param 无
     * @return　0
     */
    virtual int terminate() = 0;

    /**
     * 机械臂本体通电指令
     *
     * @param 无
     * @return　上电成功返回 RET_OK ，失败返回 RET_TIMEOUT
     */
    virtual int powerOnRobot() = 0;

    /**
     * 如果机器人电源开启，返回 true, 否则返回 false
     *
     * @return
     */
    virtual bool robotPowerStatus() = 0;

    /**
     * 如果机器人静止，返回 true，如果机器人关节还在运动，返回 false
     *
     * @return
     */
    virtual bool isRobotSteady() = 0;

    /**
     * 重启接口板
     *
     * @return
     */
    virtual int restartInterfaceBoard() = 0;

    /**
     * 机械臂本体初始化
     *
     * 读取一些基础信息（松刹车，向接口板请求硬件伺服信息）
     *          初始化机器人之前需要先对机器人上电。伺服信息是指5ms为周期接口板发出的信息
     * @param 无
     * @return　初始化成功返回 RET_OK,失败返回 false
     */
    virtual int bringUpRobot() = 0;

    /**
     * 机械臂本体断电
     *
     * @param 无
     * @return　成功返回 0 ，失败返回 -1
     */
    virtual int powerOffRobot() = 0;

    /**
     * 机械臂本体刹车释放
     *
     * @param 无
     * @return　成功返回 0 ，失败返回 -1
     */
    virtual int releaseJointBrakes() = 0;

    /**
     * 机械臂本体刹车抱死
     *
     * @param 无
     * @return　成功返回 0 ，失败返回 -1
     */
    virtual int lockJointBrakes() = 0;

    /**
     * 使能电机伺服
     *
     * @param 无
     * @return　成功返回 0 ，失败返回 -1
     */
    virtual int enableMotorServo() = 0;

    /**
     * 失能电机伺服
     *
     * @param 无
     * @return　成功返回 0 ，失败返回 -1
     */
    virtual int disableMotorServo() = 0;

    /**
     * 碰撞停止
     *
     * @param  是否发生碰撞
     * @return　成功返回 0 ，失败返回 -1
     */
    virtual int collisionStop(bool is_collision) = 0;

    /**
     * 切换器人电机伺服模式
     *
     * @param const RobotServoMode &
     * @return　成功返回 0 ，失败返回 -1
     */
    virtual int switchMotorServoMode(RobotServoMode mode) = 0;

    /**
     * 更新硬件抽象层的固件
     *
     * 需要硬件抽象层自己定义固件升级包的格式，并自己解析，为了兼容不同的硬件抽象层，
     * 可以将不同的节点的固件(接口板、关节)打包到一个升级包，并配置好元数据文件
     *
     * @param 固件文件的路径
     * @return 0 更新成功
     */
    virtual int firmwareUpdate(const std::string &firmware) = 0;

    /**
     * 获取固件升级的进程
     *
     * @return std::string 代表步骤名称, double 代表进度(0~1)，完成之后，返回
     * ("", 1)
     */
    virtual std::tuple<std::string, double> getFirmwareUpdateProcess() = 0;

    /**
     * 更新关节重力力矩，用于硬件抽象层计算关节摩擦力
     *
     * @param gravity_torq
     * @return
     */
    virtual int updateGravityTorque(
        const std::vector<double> &gravity_torq) = 0;

    /**
     * 机械臂关节位置伺服接口
     *
     * @param[in] q 各关节目标位置
     * @param[in] qd 各关节目标速度（暂不支持，传入值作无效处理）
     * @param[in] qdd 各关节目标加速度（暂不支持，传入值作无效处理）
     * @return　0
     */
    virtual int positionServo(const std::vector<double> &q,
                              const std::vector<double> &qd,
                              const std::vector<double> &qdd) = 0;

    /**
     * 使能电流伺服
     *
     * @param[in] curr 各关节电流值
     * @return　0
     */
    virtual int currentServo(const std::vector<double> &curr) = 0;

    /**
     * 关节力矩伺服控制。当关节配置有力矩传感器时，可以做力矩闭环
     *
     * @param[in] torq 各关节力矩值
     * @return　0
     */
    virtual int torqueServo(const std::vector<double> &torq) = 0;

    /**
     * 使能速度伺服
     *
     * @param[in] spd 各关节速度值
     * @return　0
     */
    virtual int speedServo(const std::vector<double> &qd) = 0;

    /**
     * 周期性指令请求（需周期性调用）
     *
     * @param 无
     * @return　0
     */
    virtual int updateRequest() = 0;

    /**
     * 获取机器人伺服模式
     *
     * @param 无
     * @return　RobotServoMode::None
     */
    virtual RobotServoMode getRobotServoMode() const = 0;

    /**
     * 获取机器人错误代码
     *
     * @param 无
     * @return RobotErrorCode
     */
    virtual RobotErrorCode getRobotError() const = 0;

    /**
     * 清除机器人错误
     *
     * @return
     */
    virtual int clearRobotError() = 0;

    /**
     * 获取机器人当前关节标志接口
     *
     * @param 无
     * @return 各个关节当前模式
     */
    virtual std::vector<JointMode> getJointModes() const = 0;

    /**
     * 获取机械臂关节错误代码
     *
     * @param 无
     * @return 各个关节的JointErrorCode
     */
    virtual std::vector<JointErrorCode> getJointErrors() const = 0;

    /**
     * 获取机械臂关节硬件版本信息
     *
     * @param 无
     * @return 各个关节的硬件版本信息
     */
    virtual std::vector<int> getJointHardwareVersions() const = 0;

    /**
     * 获取机械臂关节固件版本信息
     *
     * @param 无
     * @return 各个关节的固件版本信息
     */
    virtual std::vector<int> getJointFirmwareVersions() const = 0;

    /**
     * 获取关节伺服模式
     *
     * @param 无
     * @return JointServoMode
     */
    virtual std::vector<JointServoMode> getJointServoModes() const = 0;

    /**
     * 获取关节静摩擦力矩
     *
     * @return
     */
    virtual std::vector<double> getJointStaticFriction() const = 0;

    /**
     * 获取关节粘滞摩擦力矩
     *
     * @return
     */
    virtual std::vector<double> getJointViscousFriction() const = 0;

    /**
     * 获取关节最大输出力矩(去除了摩擦力)
     *
     * @return
     */
    virtual std::vector<double> getJointMaxTorque() const = 0;

    /**
     * 获取当前所有关节的位置角度接口
     *          \TODO(louwei):
     * 需要进行关节刚度补偿(谐波减速器柔性形变以及零位偏移)
     *          关节刚度模型需要从关节或者底座中读取，另外需要考虑是否提供模型参数写入接口
     *          考虑是否加入关节零位写入接口。可能还需要考虑减速器背隙
     *
     * @param 无
     * @return 各个关节角 单位：弧度
     */
    virtual std::vector<double> getJointPositions() const = 0;

    /**
     * 获取关节位置偏移
     *
     * @param 无
     * @return 各个关节位置偏移
     */
    virtual std::vector<double> getJointPositionOffsets() const = 0;

    /**
     * 获取关节的刚度补偿（主要来自于谐波减速器柔轮）
     *
     * @param 无
     * @return 各个关节位置变形量
     */
    virtual std::vector<double> getJointPositionDeformations() const = 0;

    /**
     * 获取关节力矩(补偿摩擦力之后)
     *
     * @return
     */
    virtual std::vector<double> getJointTorques() const = 0;

    /**
     * 获取关节减速之后转矩系数  Nm/A
     *
     * @return
     */
    virtual std::vector<double> getJointTorqueConstants() const = 0;

    /**
     * 获取当前机械臂所有关节速度接口
     *
     * @param 无
     * @return 各个关节速度
     */
    virtual std::vector<double> getJointSpeeds() const = 0;

    /**
     * 获取当前机械臂所有关节加速度接口
     *
     * @param 无
     * @return 各个关节加速度
     */
    virtual std::vector<double> getJointAccelerations() const = 0;

    /**
     * 获取机械臂关节力传感器的量程
     *
     * @param 无
     * @return 各个关节力传感器分辨率
     */
    virtual std::vector<double> getJointTorqueSensorScale() const = 0;

    /**
     * 获取机械臂关节力矩接口
     *
     * @param 无
     * @return 各个关节力传感器数值
     */
    virtual std::vector<double> getJointTorqueSensorDatas() const = 0;

    virtual std::string getToolUniqueId() const = 0;
    virtual std::string getPedestalUniqueId() const = 0;
    virtual int getPedestalFirmwareVersion() const = 0;
    virtual int getPedestalHardwareVersion() const = 0;

    /**
     * 获取末端硬件版本信息
     *
     * @param 无
     * @return 末端硬件版本信息
     */
    virtual int getToolHardwareVersion() const = 0;

    /**
     * 获取末端固件版本信息
     *
     * @param 无
     * @return 末端固件版本信息
     */
    virtual int getToolFirmwareVersion() const = 0;

    /**
     * 获取末端输入电压
     *
     * @param 无
     * @return 电压
     */
    virtual double getToolVoltage() const = 0;

    /**
     * 获取末端输入电流
     *
     * @param 无
     * @return 电流
     */
    virtual double getToolCurrent() const = 0;

    /**
     * 获取末端输出电压
     *
     * @param 无
     * @return 电压
     */
    virtual double getToolOutputVoltage() const = 0;

    /**
     * 获取关节力传感器存在标志
     *
     * @param 无
     * @return 存在 true ，不存在 false
     */
    virtual bool hasJointTorqueSensor() const = 0;

    /**
     * 获取关节 UniqueId
     *
     * @return
     */
    virtual std::vector<std::string> getJointUniqueIds() = 0;

    /**
     * 获取机械臂当前所有关节电流接口
     *
     * @param 无
     * @return 各个关节当前电流
     */
    virtual std::vector<double> getJointCurrents() const = 0;

    /**
     * 获取关节的力矩估计值（关节摩擦力模型）
     *
     * @param 无
     * @return 各个关节评估力矩
     */
    virtual std::vector<double> getJointEstimateTorques() const = 0;

    /**
     * 获取当前所有关节电压
     *
     * @param 无
     * @return 各个关节当前电压
     */
    virtual std::vector<double> getJointVoltages() const = 0;

    /**
     * 获取机械臂当前所有关节温度
     *
     * @param 无
     * @return 各个关节当前温度
     */
    virtual std::vector<double> getJointTemperatures() const = 0;

    /**
     * 获取关节最大限位（带符号）
     *
     * @param 无
     * @return 各个关节最大位置 单位：弧度
     */
    virtual std::vector<double> getJointMaxPositions() const = 0;

    /**
     * 获取关节最小限位（带符号）
     *
     * @param 无
     * @return 各个关节最小位置 单位：弧度
     */
    virtual std::vector<double> getJointMinPositions() const = 0;

    /**
     * 获取关节最大速度限制
     *
     * @param 无
     * @return 各个关节最大速度
     */
    virtual std::vector<double> getJointMaxSpeeds() const = 0;

    /**
     * 获取关节最大加速度限制
     *
     * @param 无
     * @return 各个关节最大加速度
     */
    virtual std::vector<double> getJointMaxAccelerations() const = 0;
    virtual std::vector<double> getJointMaxCurrents() const = 0;

    /**
     * 获取机械臂关节目标位置角度
     *
     * @param 无
     * @return 各个关节目标位置 单位：弧度
     */
    virtual std::vector<double> getJointTargetPositions() const = 0;

    /**
     * 获取机械臂关节目标速度
     *
     * @param 无
     * @return 各个关节目标速度
     */
    virtual std::vector<double> getJointTargetSpeeds() const = 0;

    /**
     * 获取机械臂关节目标加速度
     *
     * @param 无
     * @return 各个关节目标加速度
     */
    virtual std::vector<double> getJointTargetAccelerations() const = 0;

    /**
     * 获取机械臂关节目标力矩
     *
     * @param 无
     * @return 各个关节目标扭矩 单位：Nm
     */
    virtual std::vector<double> getJointTargetTorques() const = 0;

    /**
     * 获取机械臂关节目标电流
     *
     * @param 无
     * @return 各个关节目标电流
     */
    virtual std::vector<double> getJointTargetCurrents() const = 0;

    /**
     * 底层通讯硬件通讯（需周期性调用，功能同updateRequest）
     *
     * @param 无
     * @return　0
     */
    virtual int lowLevelCommunication() = 0;

    /**
     * 获取接口板全球唯一 ID
     *
     * @return
     */
    virtual std::string getMasterBoardUniqueId() const = 0;

    /**
     * 获取接口板主芯片硬件版本
     *
     * @param 无
     * @return 版本值
     */
    virtual int getMasterBoardHardwareVersion() const = 0;

    /**
     * 获取接口板主芯片固件版本
     *
     * @param 无
     * @return 版本值
     */
    virtual int getMasterBoardFirmwareVersion() const = 0;

    virtual std::string getSlaveBoardUniqueId() const = 0;

    /**
     * 获取接口板从芯片硬件版本
     *
     * @param 无
     * @return 版本值
     */
    virtual int getSlaveBoardHardwareVersion() const = 0;

    /**
     * 获取接口板从芯片固件版本
     *
     * @param 无
     * @return 版本值
     */
    virtual int getSlaveBoardFirmwareVersion() const = 0;

    /**
     * 获取控制柜湿度
     *
     * @param 无
     * @return 湿度
     */
    virtual double getControlBoxHumidity() const = 0;

    /**
     * 获取控制柜温度
     *
     * @param 无
     * @return 温度
     */
    virtual double getControlBoxTemperature() const = 0;

    /**
     * 获取总电压
     *
     * @param 无
     * @return 电压
     */
    virtual double getMainVoltage() const = 0;

    /**
     * 获取总电流
     *
     * @param 无
     * @return 温度
     */
    virtual double getMainCurrent() const = 0;

    /**
     * 获取机器人本体输入电压
     *
     * @param 无
     * @return 电压
     */
    virtual double getRobotVoltage() const = 0;

    /**
     * 获取机器人本体输入电流
     *
     * @param 无
     * @return 温度
     */
    virtual double getRobotCurrent() const = 0;

    /**
     * 设置参数
     *  - 动力学参数
     *  - 摩擦力参数
     *  - DH 补偿
     *
     * 实现层需要对参数字符串进行解析(toml 格式)
     *
     * @return
     */
    virtual int setPersistentParameters(const std::string &param) = 0;

    /**
     * 获取动力学参数误差
     *
     * @param 无
     * @return 动力学参数
     */
    virtual std::unordered_map<std::string, std::vector<double>>
    getDynamicsParamDeviation() const = 0;

    /**
     * 获取全动力学参数
     *
     * @param 无
     * @return 动力学参数
     */
    virtual std::vector<double> getFullDynamicsParams() const = 0;

    /**
     * 获取关节刚度模型（柔轮 flexspline）
     *
     * @param 无
     * @return 各个关节刚度系数
     */
    virtual std::vector<double> getJointStiffnessModel() const = 0;

    /**
     * 获取机器人安装姿态,如果没有加速度传感器，也需要支持手动输入
     *
     * @param 无
     * @return true-存在 false-不存在
     */
    virtual bool hasGrarvitySensor() const = 0;

    /**
     * 获取重力
     *
     * @param 无
     * @return 重力值
     */
    virtual std::vector<double> getGravity() const = 0;

    /**
     * getTcpForceSensor
     *
     * @return
     */
    virtual SensorHardwarePtr getTcpForceSensor() = 0;

    virtual SensorHardwarePtr getBaseForceSensor() = 0;

    virtual SensorHardwarePtr getBaseGrarvitySensor() = 0;

    virtual bool getTeachPendantButtonStatus() = 0;

    /**
     * Get safety input status of the robot. Generally, robot has some defined
     * safety digital inputs which will trigger emergency stop or reduced mode
     * etc
     *
     * @return
     */
    virtual SafetyMonitorStatus getSafetyMonitorStatus() = 0;

    /**
     * 获取运动学补偿参数
     *
     * @return
     */
    virtual std::unordered_map<std::string, std::vector<double>>
    getKinematicsCompensateParams() = 0;

    /**
     * By set joint to zero, defined current joint position as the new joint
     * zero position.
     *
     * @param[in] The id of the joint
     * @return
     */
    virtual int setJointZero(int joint_id) = 0;
    virtual int setJointsZero() = 0;

    ////////////////////////////////////////////////////////////////////////////////
    /// 以下接口为安全接口板相关，由于机器人本体与安全接口板一一对应，因此将两者的接口合并在一起
    /// 安全参数的范围会被保存在安全硬件中

    virtual int setSafetyParameterRange(
        const RobotSafetyParameterRange &range) = 0;
    virtual RobotSafetyParameterRange getSafetyParameterRange() = 0;

    /// 更新机器人的型号, 如果型号发生变化，所有的安全参数需要重新设置
    virtual int setRobotModelType(const std::string &type,
                                  const std::string &subtype) = 0;

    /// 更新关节状态 这些信息可以通过硬件抽象层自身获取
    // virtual int updateJointToruqes(const std::vector<double> &torqs);
    // virtual int updateJointPositions(const std::vector<double> &q) = 0;
    // virtual int updateJointSpeeds(const std::vector<double> &qd) = 0;

    /// 更新动量
    virtual int updateMomentum(double momentum) = 0;

    /// 更新末端力
    virtual int updateTcpForce(double force) = 0;

    /// 更新肘部力
    virtual int updateElbowForce(double force) = 0;

    /// 更新机器人状态到安全硬件
    virtual int updateTcpPose(const std::vector<double> &tcp_pose) = 0;
    virtual int updateTcpSpeed(const std::vector<double> &tcp_speed) = 0;
    virtual int updateElbowPose(const std::vector<double> &elbow_pose) = 0;
    virtual int updateElbowSpeed(const std::vector<double> &elbow_speed) = 0;

    /// 机器人刹车是否释放
    virtual bool robotBrakeReleased() = 0;

    /// 最多支持32路可配置IO
    virtual int getConfigurableInputNum() = 0;
    virtual int getConfigurableOutputNum() = 0;
    virtual uint32_t getConfigurableInputs() = 0;
    virtual uint32_t getConfigurableOutputs() = 0;
    virtual int setConfigurableOutput(int index, bool value) = 0;

    /// 最多支持32路用户IO
    virtual int getUserDigitalInputNum() = 0;
    virtual int getUserDigitalOutputNum() = 0;
    virtual uint32_t getUserDigitalInputs() = 0;
    virtual uint32_t getUserDigitalOutputs() = 0;
    virtual int setUserDigitalOutput(int index, bool value) = 0;

    /// User模拟IO
    virtual int getUserAnalogInputNum() = 0;
    virtual int getUserAnalogOutputNum() = 0;
    virtual double getUserAnalogInput(int index) = 0;
    virtual double getUserAnalogOutput(int index) = 0;
    virtual int setUserAnalogOutput(int index, double value) = 0;

    /// 最多支持32路工具IO
    virtual int getToolDigitalIoNum() = 0;
    virtual uint32_t getToolDigitalIo() = 0;

    /// 设置工具IO输入或者输出 0: 输出 1: 输入
    virtual int setToolDigitalIoDirection(uint32_t mask) = 0;
    virtual uint32_t getToolDigitalIoDirection() = 0;

    /// 只对输出有效
    virtual int setToolDigitalIo(int index, bool value) = 0;

    /// 设置工具端对外输出电压范围
    ///
    /// @param domain 可选三个档位 0: 0V 12:12V 24:24V
    virtual int setToolVoltageOutputDomain(int domain) = 0;
    virtual int getToolVoltageOutputDomain() = 0;

    /// 工具模拟IO
    virtual int getToolAnalogInputNum() = 0;
    virtual int getToolAnalogOutputNum() = 0;
    virtual double getToolAnalogInput(int index) = 0;
    virtual double getToolAnalogOutput(int index) = 0;
    virtual int setToolAnalogOutput(int index, double value) = 0;

    /// 联动IO
    virtual int getLinkInputNum() { return 0; }
    virtual int getLinkOutputNum() { return 0; }
    virtual uint32_t getLinkInputs() { return 0; }
    virtual uint32_t getLinkOutputs() { return 0; }

    /// 获取联动输入状态
    virtual LinkInputStatus getLinkInputStatus() { return {}; }

    /// 设置联动输出状态
    virtual int setLinkOutputStatus(LinkOutputStatus) { return 0; }
};
using RobotHardwarePtr = std::shared_ptr<RobotHardware>;

} // namespace common_interface
} // namespace arcs

#endif // COMMON_INTERFACE_ROBOTHW_H
