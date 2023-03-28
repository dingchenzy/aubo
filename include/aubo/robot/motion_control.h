#ifndef AUBO_SDK_MOTION_CONTROL_INTERFACE_H
#define AUBO_SDK_MOTION_CONTROL_INTERFACE_H

#include <vector>
#include <functional>
#include <thread>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

/**
 * @file motion_control.h
 *
 * The robot movements are programmed as pose-to-pose movements, that is move
 * from the current position to a new position. The path between these two
 * positions is then automatically calculated by the robot.
 */
namespace arcs {
namespace common_interface {

enum PathBufferType
{
    PathBuffer_TOPPRA = 1,      // 1: toppra 时间最优路径规划
    PathBuffer_CubicSpline = 2, // 2: cubic_spline(录制的轨迹)
    PathBuffer_JointSpline = 3, // 3: 关节B样条插值，最少三个点
};

/**
 *  The MotionControl class
 */
class ARCS_ABI_EXPORT MotionControl
{
public:
    MotionControl();
    virtual ~MotionControl();

    /**
     * 获取等效半径，单位 m
     * moveLine/moveCircle时，末端姿态旋转的角度等效到末端位置移动
     *
     * @return 返回等效半径
     *
     * @code Python函数原型
     * getEqradius(self: pyaubo_sdk.MotionControl) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getEqradius() -> number
     * @endcode
     */
    double getEqradius();

    /**
     * 动态调整机器人运行速度和加速度比例 (0., 1.]
     *
     * @param fraction 机器人运行速度和加速度比例
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setSpeedFraction(self: pyaubo_sdk.MotionControl, arg0: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setSpeedFraction(fraction: number) -> nil
     * @endcode
     */
    int setSpeedFraction(double fraction);

    /**
     * 获取速度和加速度比例，默认为 1
     * 可以通过 setSpeedFraction 接口设置
     *
     * @return 返回速度和加速度比例
     *
     * @code Python函数原型
     * getSpeedFraction(self: pyaubo_sdk.MotionControl) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getSpeedFraction() -> number
     * @endcode
     */
    double getSpeedFraction();

    /**
     * 路径偏移使能
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * pathOffsetEnable(self: pyaubo_sdk.MotionControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pathOffsetEnable() -> number
     * @endcode
     */
    int pathOffsetEnable();

    /**
     * 设置路径偏移
     *
     * @param offset 在各方向的位姿偏移
     * @param type
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * pathOffsetSet(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pathOffsetSet(offset: table, type: number) -> nil
     * @endcode
     */
    int pathOffsetSet(const std::vector<double> &offset, int type = 1);

    /**
     * 路径偏移失能
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * pathOffsetDisable(self: pyaubo_sdk.MotionControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pathOffsetDisable() -> nil
     * @endcode
     */
    int pathOffsetDisable();

    /**
     * 关节偏移使能
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * jointOffsetEnable(self: pyaubo_sdk.MotionControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * jointOffsetEnable() -> nil
     * @endcode
     */
    int jointOffsetEnable();

    /**
     * 设置关节偏移
     *
     * @param offset 在各关节的位姿偏移
     * @param type
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * jointOffsetSet(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * jointOffsetSet(offset: table, type: number) -> nil
     * @endcode
     */
    int jointOffsetSet(const std::vector<double> &offset, int type = 1);

    /**
     * 关节偏移失能
     *
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * jointOffsetDisable(self: pyaubo_sdk.MotionControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * jointOffsetDisable() -> nil
     * @endcode
     */
    int jointOffsetDisable();

    /**
     * 获取已经入队的运动指令段数量
     *
     * @return 已经入队的运动指令段数量
     *
     * @code Python函数原型
     * getQueueSize(self: pyaubo_sdk.MotionControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getQueueSize() -> number
     * @endcode
     */
    int getQueueSize();

    /**
     * 获取已经入队的运动规划插补点数量
     *
     * @return 已经入队的运动规划插补点数量
     *
     * @code Python函数原型
     * getTrajectoryQueueSize(self: pyaubo_sdk.MotionControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getTrajectoryQueueSize() -> number
     * @endcode
     */
    int getTrajectoryQueueSize();

    /**
     * 获取当前正在插补的运动指令段的ID
     *
     * @return 当前正在插补的运动指令段的ID
     * @retval -1 表示轨迹队列为空 \n
     * 像movePathBuffer运动中的buffer或者规划器(moveJoint和moveLine等)里的队列都属于轨迹队列.\n
     *
     * @code Python函数原型
     * getExecId(self: pyaubo_sdk.MotionControl) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getExecId() -> number
     * @endcode
     */
    int getExecId();

    /**
     * 获取指定ID的运动指令段的预期执行时间
     *
     * @param id 运动指令段ID
     * @return 返回预期执行时间
     *
     * @code Python函数原型
     * getDuration(self: pyaubo_sdk.MotionControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getDuration(id: number) -> number
     * @endcode
     */
    double getDuration(int id);

    /**
     * 获取指定ID的运动指令段的剩余执行时间
     *
     * @param id 运动指令段ID
     * @return 返回剩余执行时间
     *
     * @code Python函数原型
     * getMotionLeftTime(self: pyaubo_sdk.MotionControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getMotionLeftTime(id: number) -> number
     * @endcode
     */
    double getMotionLeftTime(int id);

    /**
     * 获取当前运动指令段的执行进度
     *
     * @return 返回执行进度
     *
     * @code Python函数原型
     * getProgress(self: pyaubo_sdk.MotionControl) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getProgress() -> number
     * @endcode
     */
    double getProgress();

    /**
     * 当工件安装在另外一台机器人的末端或者外部轴上时，指定其名字和安装位置
     *
     * @param module_name 控制模块名字
     * @param mounting_pose 抓取的相对位置，
     * 如果是机器人，则相对于机器人末端中心点（非TCP点）
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * setWorkObjectHold(self: pyaubo_sdk.MotionControl, arg0: str, arg1:
     * List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setWorkObjectHold(module_name: string, mounting_pose: table) -> nil
     * @endcode
     */
    int setWorkObjectHold(const std::string &module_name,
                          const std::vector<double> &mounting_pose);

    /**
     * getWorkObjectHold
     *
     * @note 暂未实现
     *
     * @return
     *
     * @code Python函数原型
     * getWorkObjectHold(self: pyaubo_sdk.MotionControl) -> Tuple[str,
     * List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * getWorkObjectHold() -> table
     * @endcode
     */
    std::tuple<std::string, std::vector<double>> getWorkObjectHold();

    /**
     * getPauseJointPositions
     *
     * @note 获取暂停点关节位置
     * 常用于运行工程时发生碰撞后继续运动过程中(先通过resumeMoveJoint或resumeMoveLine运动到暂停位置,再恢复工程)
     *
     * @return
     * 暂停关节位置
     *
     * @code Python函数原型
     *
     *
     * @endcode
     *
     * @code Lua函数原型
     *
     * @endcode
     */
    std::vector<double> getPauseJointPositions();

    /**
     *
     * @param enable
     * @return
     *
     * @code Python函数原型
     * setServoMode(self: pyaubo_sdk.MotionControl, arg0: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setServoMode(enable: boolean) -> nil
     * @endcode
     */
    int setServoMode(bool enable);

    /**
     *
     * @return
     *
     * @code Python函数原型
     * isServoModeEnabled(self: pyaubo_sdk.MotionControl) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * isServoModeEnabled() -> boolean
     * @endcode
     */
    bool isServoModeEnabled();

    /**
     * 关节空间伺服
     *
     * @note 暂未实现
     *
     * @code Python函数原型
     * servoJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float, arg4: float, arg5: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * servoJoint(q: table, a: number, v: number, t: number, lookahead_time:
     * number, gain: number) -> nil
     * @endcode
     */
    int servoJoint(const std::vector<double> &q, double a, double v, double t,
                   double lookahead_time, double gain);

    /**
     * 关节空间跟随
     *
     * @note 暂未实现
     *
     * @code Python函数原型
     * followJoint(self: pyaubo_sdk.MotionControl, arg0: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * followJoint(q: table) -> nil
     * @endcode
     */
    int followJoint(const std::vector<double> &q);

    /**
     * 笛卡尔空间跟随
     *
     * @note 暂未实现
     *
     * @code Python函数原型
     * followLine(self: pyaubo_sdk.MotionControl, arg0: List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * followLine(pose: table) -> nil
     * @endcode
     */
    int followLine(const std::vector<double> &pose);

    /**
     * 关节空间速度跟随
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param q 目标关节速度, 单位 rad/s
     * @param a 主轴的加速度, 单位 rad/s^2
     * @param t 函数返回所需要的时间, 单位 s
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * speedJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * speedJoint(q: table, a: number, t: number) -> nil
     * @endcode
     */
    int speedJoint(const std::vector<double> &qd, double a, double t);

    /**
     * 关节空间速度跟随(机械臂运行工程时发生碰撞,通过此接口移动到安全位置)
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param qd 目标关节速度, 单位 rad/s
     * @param a 主轴的加速度, 单位 rad/s^2
     * @param t 函数返回所需要的时间, 单位 s
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * resumeSpeedJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * resumeSpeedJoint(q: table, a: number, t: number) -> nil
     * @endcode
     */
    int resumeSpeedJoint(const std::vector<double> &qd, double a, double t);

    /**
     * 笛卡尔空间速度跟随
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param xd 工具速度, 单位 m/s
     * @param a 工具位置加速度, 单位 m/s^2
     * @param t 函数返回所需要的时间, 单位 s
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * speedLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1: float,
     * arg2: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * speedLine(pose: table, a: number, t: number) -> nil
     * @endcode
     */
    int speedLine(const std::vector<double> &xd, double a, double t);

    /**
     * 笛卡尔空间速度跟随(机械臂运行工程时发生碰撞,通过此接口移动到安全位置)
     * 当机械臂还没达到目标速度的时候，给一个新的目标速度，机械臂会立刻达到新的目标速度
     *
     * @param xd 工具速度, 单位 m/s
     * @param a 工具位置加速度, 单位 m/s^2
     * @param t 函数返回所需要的时间, 单位 s
     * 如果 t = 0，当达到目标速度的时候，函数将返回；
     * 反之，则经过 t 时间后，函数返回，不管是否达到目标速度。
     * 如果没有达到目标速度，会减速到零。
     * 如果达到了目标速度就是按照目标速度匀速运动。
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * resumeSpeedLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * resumeSpeedLine(pose: table, a: number, t: number) -> nil
     * @endcode
     */
    int resumeSpeedLine(const std::vector<double> &xd, double a, double t);

    /**
     * 在关节空间做样条插值
     *
     * @param q 关节角度，如果传入参数维度为0，表示样条运动结束
     * @param a 加速度, 单位 rad/s^2
     * @param v 速度， 单位 rad/s
     * @param duration
     * @return 成功返回0; 失败返回错误码
     *
     * @code Python函数原型
     * moveSpline(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * moveSpline(q: table, a: number, v: number, duration: number) -> nil
     * @endcode
     */
    int moveSpline(const std::vector<double> &q, double a, double v,
                   double duration);

    /**
     * 添加关节运动
     *
     * @param q 关节角, 单位 rad
     * @param a 加速度, 单位 rad/s^2
     * @param v 速度， 单位 rad/s
     * @param blend_radius 交融半径
     * @param duration 运行时间，单位 s
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。
     * 当 duration = 0
     * 的时候，表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * moveJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1: float,
     * arg2: float, arg3: float, arg4: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * moveJoint(q: table, a: number, v: number, blend_radius: number, duration:
     * number) -> nil
     * @endcode
     */
    int moveJoint(const std::vector<double> &q, double a, double v,
                  double blend_radius, double duration);

    /**
     * 通过关节运动移动到暂停点的位置
     *
     * @param q 关节角, 单位 rad
     * @param a 加速度, 单位 rad/s^2
     * @param v 速度， 单位 rad/s
     * @param duration 运行时间，单位 s
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。
     * 当 duration = 0
     * 的时候，表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * resumeMoveJoint(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * resumeMoveJoint(q: table, a: number, v: number, duration: number) -> nil
     * @endcode
     */
    int resumeMoveJoint(const std::vector<double> &q, double a, double v,
                        double duration);

    /**
     * 添加直线运动
     *
     * @param pose 目标位姿
     * @param a 加速度, 单位 m/s^2
     * @param v 速度, 单位 m/s
     * @param blend_radius 交融半径
     * @param duration 运行时间，单位 s
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。
     * 当 duration = 0
     * 的时候，表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * moveLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1: float,
     * arg2: float, arg3: float, arg4: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * moveLine(pose: table, a: number, v: number, blend_radius: number,
     * duration: number) -> nil
     * @endcode
     */
    int moveLine(const std::vector<double> &pose, double a, double v,
                 double blend_radius, double duration);

    /**
     * 通过直线运动移动到暂停点的位置
     *
     * @param pose 目标位姿
     * @param a 加速度, 单位 m/s^2
     * @param v 速度, 单位 m/s
     * @param duration 运行时间，单位 s
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。
     * 当 duration = 0
     * 的时候，表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * resumeMoveLine(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * float, arg2: float, arg3: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * resumeMoveLine(pose: table, a: number, v: number,duration: number) -> nil
     * @endcode
     */
    int resumeMoveLine(const std::vector<double> &pose, double a, double v,
                       double duration);

    /**
     * 添加圆弧运动
     *
     * @todo 可以由多段圆弧组成圆周运动
     *
     * @param via_pose 圆弧运动途中点的位姿
     * @param end_pose 圆弧运动结束点的位姿
     * @param a 加速度, 单位: m/s^2
     * @param v 速度，单位: m/s
     * @param blend_radius 交融半径
     * @param duration 运行时间，单位 s
     * 通常当给定了速度和加速度，便能够确定轨迹的运行时间。
     * 如果想延长轨迹的运行时间，便要设置 duration 这个参数。
     * duration 可以延长轨迹运动的时间，但是不能缩短轨迹时间。
     * 当 duration = 0
     * 的时候，表示不指定运行时间，即没有明确完成运动的时间，将根据速度与加速度计算运行时间。
     * 如果duration不等于0，a 和 v 的值将被忽略。
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * moveCircle(self: pyaubo_sdk.MotionControl, arg0: List[float], arg1:
     * List[float], arg2: float, arg3: float, arg4: float, arg5: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * moveCircle(via_pose: table, end_pose: table, a: number, v: number,
     * blend_radius: number, duration: number) -> nil
     * @endcode
     */
    int moveCircle(const std::vector<double> &via_pose,
                   const std::vector<double> &end_pose, double a, double v,
                   double blend_radius, double duration);

    /**
     *
     * 设置圆弧路径模式
     *
     * @param mode
     *          0:工具姿态相对于圆弧路径点坐标系保持不变
     *          1:姿态线性变化,绕着空间定轴转动,从起始点姿态变化到目标点姿态
     *          2:从起点姿态开始经过中间点姿态,变化到目标点姿态
     *
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * setCirclePathMode(self: pyaubo_sdk.MotionControl, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setCirclePathMode(mode: number) -> nil
     * @endcode
     */
    int setCirclePathMode(int mode);

    /**
     * 高级圆弧或者圆周运动
     *
     * @param param 圆周运动参数
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * moveCircle2(self: pyaubo_sdk.MotionControl, arg0:
     * arcs::common_interface::CircleParameters) -> int
     * @endcode
     *
     * @code Lua函数原型
     * moveCircle2(param: table) -> nil
     * @endcode
     */
    int moveCircle2(const CircleParameters &param);

    /**
     * 新建一个路径点缓存
     *
     * @param name 指定路径的名字
     * @param type 路径的类型
     *   1: toppra 时间最优路径规划
     *   2: cubic_spline(录制的轨迹)
     *   3: 关节B样条插值，最少三个点
     * @param size 缓存区大小
     * @return 新建成功返回 AUBO_OK(0)
     *
     * @code Python函数原型
     * pathBufferAlloc(self: pyaubo_sdk.MotionControl, arg0: str, arg1: int,
     * arg2: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pathBufferAlloc(name: string, type: number, size: number) -> nil
     * @endcode
     */
    int pathBufferAlloc(const std::string &name, int type, int size);

    /**
     * 向路径缓存添加路点
     *
     * @param name 路径缓存的名字
     * @param waypoints 路点
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * pathBufferAppend(self: pyaubo_sdk.MotionControl, arg0: str, arg1:
     * List[List[float]]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pathBufferAppend(name: string, waypoints: table) -> nil
     * @endcode
     */
    int pathBufferAppend(const std::string &name,
                         const std::vector<std::vector<double>> &waypoints);

    /**
     * 计算、优化等耗时操作，传入的参数相同时不会重新计算
     *
     * @param name 通过pathBufferAlloc新建的路径点缓存的名字
     * @param a 关节加速度限制
     * @param v 关节速度限制
     * @param t 时间，共有2种类型.
     * pathBufferAlloc 这个接口分配内存的时候要指定类型，
     * 根据pathBufferAlloc这个接口的类型:\n
     * 类型为1时,表示运动持续时间\n
     * 类型为2时,表示采样时间间隔\n
     * 类型为3时,t参数设置为0
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * pathBufferEval(self: pyaubo_sdk.MotionControl, arg0: str, arg1:
     * List[float], arg2: List[float], arg3: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pathBufferEval(name: string, a: table, v: table, t: number) -> nil
     * @endcode
     */
    int pathBufferEval(const std::string &name, const std::vector<double> &a,
                       const std::vector<double> &v, double t);

    /**
     * 指定名字的buffer是否有效
     *
     * buffer需要满足三个条件有效: \n
     * 1、buffer存在，已经分配过内存 \n
     * 2、传进buffer的点要大于等于buffer的大小 \n
     * 3、要执行一次pathBufferEval
     *
     * @param name buffer的名字
     * @return 有效返回true; 无效返回false
     *
     * @code Python函数原型
     * pathBufferValid(self: pyaubo_sdk.MotionControl, arg0: str) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * pathBufferValid(name: string) -> boolean
     * @endcode
     */
    bool pathBufferValid(const std::string &name);

    /**
     * 释放路径缓存
     *
     * @param name 缓存路径的名字
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * pathBufferFree(self: pyaubo_sdk.MotionControl, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * pathBufferFree(name: string) -> nil
     * @endcode
     */
    int pathBufferFree(const std::string &name);

    /**
     * 列出所有缓存路径的名字
     *
     * @return 返回所有缓存路径的名字
     *
     * @code Python函数原型
     * pathBufferList(self: pyaubo_sdk.MotionControl) -> List[str]
     * @endcode
     *
     * @code Lua函数原型
     * pathBufferList() -> table
     * @endcode
     */
    std::vector<std::string> pathBufferList();

    /**
     * 执行缓存的路径
     *
     * @param name 缓存路径的名字
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * movePathBuffer(self: pyaubo_sdk.MotionControl, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * movePathBuffer(name: string) -> nil
     * @endcode
     */
    int movePathBuffer(const std::string &name);

    /**
     * 关节空间停止运动
     *
     * @param acc 关节加速度，单位: rad/s^2
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * stopJoint(self: pyaubo_sdk.MotionControl, arg0: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * stopJoint(acc: number) -> nil
     * @endcode
     */
    int stopJoint(double acc);

    /**
     * 关节空间停止运动(机械臂运行工程时发生碰撞,通过resumeSpeedJoint接口移动到安全位置后需要停止时调用此接口)
     *
     * @param acc 关节加速度，单位: rad/s^2
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * resumeStopJoint(self: pyaubo_sdk.MotionControl, arg0: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * resumeStopJoint(acc: number) -> nil
     * @endcode
     */
    int resumeStopJoint(double acc);

    /**
     * 笛卡尔空间停止运动
     *
     * @param acc 工具加速度, 单位: m/s^2
     * @param acc_rot
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * stopLine(self: pyaubo_sdk.MotionControl, arg0: float, arg1: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * stopLine(acc: number, acc_rot: number) -> nil
     * @endcode
     */
    int stopLine(double acc, double acc_rot);

    /**
     * 笛卡尔空间停止运动(机械臂运行工程时发生碰撞,通过resumeSpeedLine接口移动到安全位置后需要停止时调用此接口)
     *
     * @param acc 工具加速度, 单位: m/s^2
     * @param acc_rot
     * @return 成功返回0；失败返回错误码
     *
     * @code Python函数原型
     * resumeStopLine(self: pyaubo_sdk.MotionControl, arg0: float, arg1: float)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * resumeStopLine(acc: number, acc_rot: number) -> nil
     * @endcode
     */
    int resumeStopLine(double acc, double acc_rot);

protected:
    void *d_;
};
using MotionControlPtr = std::shared_ptr<MotionControl>;

// clang-format off
#define MotionControl_DECLARES                                                  \
    _FUNC(MotionControl, 0, getEqradius)                                        \
    _FUNC(MotionControl, 0, getSpeedFraction)                                   \
    _FUNC(MotionControl, 1, setSpeedFraction, fraction)                         \
    _INST(MotionControl, 0, pathOffsetEnable)                                   \
    _INST(MotionControl, 2, pathOffsetSet, offset, type)                        \
    _INST(MotionControl, 0, pathOffsetDisable)                                  \
    _INST(MotionControl, 0, jointOffsetEnable)                                  \
    _INST(MotionControl, 2, jointOffsetSet, offset, type)                       \
    _INST(MotionControl, 0, jointOffsetDisable)                                 \
    _FUNC(MotionControl, 0, getTrajectoryQueueSize)                             \
    _FUNC(MotionControl, 0, getQueueSize)                                       \
    _FUNC(MotionControl, 0, getExecId)                                          \
    _FUNC(MotionControl, 1, getDuration, id)                                    \
    _FUNC(MotionControl, 1, getMotionLeftTime, id)                              \
    _FUNC(MotionControl, 0, getProgress)                                        \
    _INST(MotionControl, 2, setWorkObjectHold, module_name, mounting_pose)      \
    _FUNC(MotionControl, 0, getWorkObjectHold)                                  \
    _FUNC(MotionControl, 0, getPauseJointPositions)                             \
    _FUNC(MotionControl, 1, setServoMode, enable)                               \
    _FUNC(MotionControl, 0, isServoModeEnabled)                                 \
    _FUNC(MotionControl, 6, servoJoint, q, a, v, t, lookahead_time, gain)       \
    _INST(MotionControl, 1, followJoint, q)                                     \
    _INST(MotionControl, 1, followLine, pose)                                   \
    _INST(MotionControl, 3, speedJoint, qd, a, t)                               \
    _FUNC(MotionControl, 3, resumeSpeedJoint, qd, a, t)                         \
    _INST(MotionControl, 3, speedLine, xd, a, t)                                \
    _FUNC(MotionControl, 3, resumeSpeedLine, xd, a, t)                          \
    _INST(MotionControl, 4, moveSpline, q, a, v, duration)                      \
    _INST(MotionControl, 5, moveJoint, q, a, v, blend_radius, duration)         \
    _FUNC(MotionControl, 4, resumeMoveJoint, q, a, v, duration)   \
    _INST(MotionControl, 5, moveLine, pose, a, v, blend_radius, duration)       \
    _FUNC(MotionControl, 4, resumeMoveLine, pose, a, v, duration) \
    _INST(MotionControl, 6, moveCircle, via_pose, end_pose, a, v,               \
                                blend_radius, duration)                         \
    _INST(MotionControl, 1, setCirclePathMode, mode)                            \
    _INST(MotionControl, 1, moveCircle2, param)                                 \
    _FUNC(MotionControl, 3, pathBufferAlloc, name, type, size)                  \
    _FUNC(MotionControl, 2, pathBufferAppend, name, waypoints)                  \
    _FUNC(MotionControl, 4, pathBufferEval, name, a, v, t)                      \
    _FUNC(MotionControl, 1, pathBufferValid, name)                              \
    _FUNC(MotionControl, 1, pathBufferFree, name)                               \
    _FUNC(MotionControl, 0, pathBufferList)                                     \
    _INST(MotionControl, 1, movePathBuffer, name)                               \
    _FUNC(MotionControl, 1, stopJoint, acc)                                     \
    _FUNC(MotionControl, 1, resumeStopJoint, acc)                               \
    _FUNC(MotionControl, 2, stopLine, acc, acc_rot)                             \
    _FUNC(MotionControl, 2, resumeStopLine, acc, acc_rot)
// clang-format on
} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_MOTION_CONTROL_INTERFACE_H
