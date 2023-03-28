#ifndef COMMON_INTERFACE_TP_H
#define COMMON_INTERFACE_TP_H

#include <vector>
#include <memory>
#include <iostream>
#include <functional>

namespace arcs {
namespace common_interface {

class TrajectoryPlan;
using TrajectoryPlanPtr = std::shared_ptr<TrajectoryPlan>;

class RobotModel;
using RobotModelPtr = std::shared_ptr<RobotModel>;

struct Trajectory
{
    /// 轨迹类型
    enum Type
    {
        None,
        Joint,
        Cartesian,
        JointAndCartesian,
    };

    bool stopping{ false }; ///< 轨迹点是否为规划暂停或停止产生
    int id{ -1 };           ///< consecutively increasing ID
    Type type{ None };      ///< type of the trajectory
    double time;            ///< struct timeval stamp;
    double progress;        ///< 执行的进度
    std::vector<double> x, xd, xdd; ///< 笛卡尔空间
    std::vector<double> q, qd, qdd; ///< 关节空间
    std::vector<double> effort;     ///< 力控
    std::function<bool()> action; ///< 用户自定义动作: IO setTcp setPayload
};

struct Path
{
    typedef std::shared_ptr<Path> Ptr;

    /// Path类型
    enum Type
    {
        PTP,     ///< 点对点运动
        LINE,    ///< 直线运动
        CIRCLE,  ///< 圆周运动
        SPEEDJ,  ///< 关节空间速度运动
        SPEEDL,  ///< 笛卡尔空间速度运动
        EXTAXIS, ///< 外部轴运动
    };

    /// 圆弧路径姿态插补方式
    enum CirPathMode
    {
        PathFram, ///< The PathFrame mode makes it easy to get
                  ///< the same angle of the tool around the cylinder. The robot
                  ///< wrist will not go through the programmed orientation in
                  ///< the ViaPoint
        ObjectFrame, ///< This mode will make a linear reorientation of the
                     ///< tool in the same way as for MoveL. The robot wrist
                     ///< will not go through the programmed orientation
                     ///< in the ViaPoint.
        ViaPointOri, ///< The ViaPointOri mode will make the robot
                     ///< wrist to go through the programmed orientation
                     ///< in the ViaPoint.
                     ///< The path is always the same in xyz but the orientation
                     ///< is different.
        Wrist45,     ///< It is assumed that the cutting beam is aligned
                     ///< with the tool’s z axis. The coordinate frame of
                     ///< the cut plane is defined by the robot’s starting
                     ///< position when executing the MoveC instruction.

        Wrist46,
        Wrist55,
    };

    RobotModelPtr robot; ///< 机器人模型，用于配置多机器人同步运动
    Type type;           ///< 路径类型
    int id;              ///< 路径段ID
    std::vector<double> target; ///< 目标
                                ///< PTP: 对应关节角度
                                ///< LINE: 对应位姿
                                ///< CIRCLE: via_pose + to_pose
    double a;                   ///< 最大加速度
    double v;                   ///< 最大速度
    double blend_radius;        ///< 交融半径
    double duration;            ///< 运动时间放大
    CirPathMode cir_mode;       ///< 姿态插补模式: CIRCLE运动有效
    std::vector<Ptr> sync_path; ///< 同步路径
};

/**
 * 轨迹规划器返回值
 * < 0 错误
 * = 0 成功
 * > 0 警告(提示)
 */
enum TPResult
{
    TP_POSE_SELF_COLLISION = -12,   ///< 暂停点不匹配
    TP_UNMATCH_PAUSE_POINT = -11,   ///< 暂停点不匹配
    TP_KIN_IK_NO_SOLU = -10,        ///< 没有解
    TP_KIN_IK_Q_REF_OUT_BOUND = -9, ///< 参考关节角超关节限制
    TP_POSE_OUT_OF_REACH = -8,      ///< 超出运动范围
    TP_JOINT_OVER_ACC = -7,         ///< 关节加速度超限制
    TP_JOINT_OVER_SPEED = -6,       ///< 关节速度超限制
    TP_SINGULARITY_ERROR = -5,      ///< 奇异点
    TP_EXECUTE_FAILED = -4,         ///< 执行失败
    TP_PLAN_FAILED = -3,            ///< 规划失败
    TP_ILEGAL_ARGS = -1,            ///< 非法参数
    TP_OK = 0,                      ///< 成功
    TP_QUEUE_FULL = 1,              ///< 队列满
    TP_EXECUTE_COMPLETE = 2,        ///< 执行完成
    TP_IGNORE = 3,                  ///< 路径段可被忽略
    TP_TOO_SHORT = 4,               ///< 目标点之间的距离太近
};

inline std::ostream &operator<<(std::ostream &os, TPResult v)
{
#undef CASE
#define CASE(v)       \
    case TPResult::v: \
        os << #v;     \
        break;

    switch (v) {
        CASE(TP_POSE_SELF_COLLISION);
        CASE(TP_UNMATCH_PAUSE_POINT);
        CASE(TP_KIN_IK_NO_SOLU);
        CASE(TP_KIN_IK_Q_REF_OUT_BOUND);
        CASE(TP_POSE_OUT_OF_REACH);
        CASE(TP_JOINT_OVER_ACC);
        CASE(TP_JOINT_OVER_SPEED);
        CASE(TP_SINGULARITY_ERROR);
        CASE(TP_EXECUTE_FAILED);
        CASE(TP_PLAN_FAILED);
        CASE(TP_TOO_SHORT);
        CASE(TP_ILEGAL_ARGS);
        CASE(TP_OK);
        CASE(TP_QUEUE_FULL);
        CASE(TP_EXECUTE_COMPLETE);
        CASE(TP_IGNORE);
    }
    return os;
}

/**
 * 软件层会创建多个规划器，实现暂停、恢复
 */
class TrajectoryPlan
{
public:
    enum PlanStatus
    {
        IDLE = 0,
        RUNNING = 1,
        PAUSING = 2,
        PAUSED = 3,
        RESUMING = 4,
        STOPPING = 5,
    };

    virtual ~TrajectoryPlan() = default;

    /**
     * Initialize the planner with robot model instance
     *
     * @param robot_model
     * @return
     */
    virtual int init(RobotModelPtr robot_model, double cycle_time) = 0;

    /**
     * Clear all path segments in the planner and set the start point
     *
     * @param initpoint
     * @return
     */
    virtual int reset(const Trajectory &initpoint, bool force = false) = 0;

    virtual int setJointMaxVelocities(const std::vector<double> &maxvel) = 0;
    virtual int setJointMaxAccelerations(const std::vector<double> &maxacc) = 0;
    virtual int setTcpMaxVelocities(double v, double vrot) = 0;
    virtual int setTcpMaxAccelerations(double a, double arot) = 0;

    virtual int setSpeedFraction(double f) = 0;

    /// 获取队列长度
    virtual int getQueueSize() = 0;

    virtual int setTargetTcpOffset(int id,
                                   const std::vector<double> &offset) = 0;

    /// 添加一段路径, 可以添加同步路径
    virtual int addPathSegment(const Path::Ptr &path) = 0;

    /// 返回路径段ID
    virtual int jointMoveTo(int id, const std::vector<double> &q, double v,
                            double a, double radius, double duration) = 0;
    virtual int lineMoveTo(int id, const std::vector<double> &pose_to, double v,
                           double a, double radius, double duration) = 0;
    virtual int circleMoveTo(int id, const std::vector<double> &pose_via,
                             const std::vector<double> &pose_to, double v,
                             double a, double radius, double duration,
                             int mode) = 0;
    virtual int circleMoveTo(int id, const std::vector<double> &pose_via,
                             const std::vector<double> &pose_to, double v,
                             double a, double radius, double duration,
                             double helix, double spiral, double direction,
                             int mode, int loop_times) = 0;
    virtual int processMove(int id,
                            const std::vector<std::vector<double>> &pose,
                            double v, double a, double radius,
                            double duration) = 0;
    virtual int splineMove(int id,
                           const std::vector<std::vector<double>> &joints,
                           double v, double a, double duration) = 0;

    virtual int servoJoint(int id, const std::vector<double> &q, double a,
                           double v, double lookahead_time, double duration,
                           double gain) = 0;

    virtual int speedJoint(int id, const std::vector<double> &qd, double a,
                           double duration) = 0;

    // 各关节以最各自加速度减速  不同步
    virtual int stopJoint(int id, double a) = 0;
    virtual int speedLine(int id, const std::vector<double> &xd, double a,
                          double time) = 0;
    virtual int stopLine(int id, double a, double arot) = 0;
    virtual int speedCircle(int id, const std::vector<double> &xd, double a,
                            double time) = 0;

    virtual int interpJoint(int id, const std::vector<double> &q) = 0;
    virtual int interpJointFinish() = 0;

    /// 运行离线轨迹，给定关节角度
    virtual int moveTrajectory(int id,
                               const std::vector<std::vector<double>> &q) = 0;

    /**
     * 周期性计算
     * 返回正在执行的路径段ID
     * 返回轨迹点
     * 返回剩余时间
     * 路径段是否发生切换
     * 错误代码
     */
    virtual int updateCycle(Trajectory &target, double elapse) = 0;

    /**
     * 获取当前运动进度
     *
     * @return
     */
    virtual double getMoveProcess() = 0;

    /**
     * 获取最后的规划目标点
     *
     * @param level
     *    0: rsCalJointCommand 输出的目标点(默认)
     *    1: TP 输出的目标点
     * @return
     */
    virtual Trajectory getLastTrajectory(int level = 0) = 0;

    virtual void clearLastTrajectory() = 0;

    virtual int abort() = 0;
    virtual int stop() = 0;
    virtual int pause() = 0;

    virtual std::vector<double> getPauseJointPositions() = 0;

    virtual PlanStatus getPlanStatus() = 0;

    /// 从指定路点恢复
    virtual int resume(const std::vector<double> &q) = 0;

    /// 生成轨迹点
    virtual std::vector<std::vector<double>> toppra(
        const std::vector<std::vector<double>> &q, const std::vector<double> &a,
        const std::vector<double> &v, double t) = 0;

    /// 样条差值
    /// q: 采样关节角度
    /// sample_time: 采样时间间隔
    virtual std::vector<std::vector<double>> cubicInterplote(
        const std::vector<std::vector<double>> &q, double sample_time) = 0;

    /// 关节B样条差值
    /// q: 采样关节角度
    /// sample_time: 采样时间间隔
    virtual int bsplineInterplote(
        int id, const std::vector<std::vector<double>> &q) = 0;
};

} // namespace common_interface
} // namespace arcs

#endif // ARAL_EXPORT_TP_H
