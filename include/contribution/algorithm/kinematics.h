#ifndef COMMON_INTERFACE_KINEMATICS_H
#define COMMON_INTERFACE_KINEMATICS_H

#include <vector>
#include <memory>

namespace arcs {
namespace common_interface {

class KinematicsSolver;
using KinematicsSolverPtr = std::shared_ptr<KinematicsSolver>;
class RobotModel;
using RobotModelPtr = std::shared_ptr<RobotModel>;

/**
 * 运动学解算器
 */
class KinematicsSolver
{
public:
    /// 机械臂运动学参数辨识输入
    struct KinCalibPara
    {
        std::vector<double> tool_para;
        std::vector<double> measurement_para;
        std::vector<double> measurement_rpy_para;
        std::vector<double> dh; // order: alhpa, a, d, theta
        std::vector<double> beta;
    };

    /// 机械臂运动学参数辨识结果
    struct KinCalibResult
    {
        KinCalibPara para_calid;
        std::vector<double> all_d_para;
        double mean_value;
        double max_value;
        double rms_value;
    };

    virtual ~KinematicsSolver() = default;

    virtual int forwardPosition(RobotModelPtr robot_model,
                                const std::vector<double> &qin,
                                std::vector<double> &frame) = 0;
    virtual int forwardToolPosition(RobotModelPtr robot_model,
                                    const std::vector<double> &qin,
                                    std::vector<double> &frame) = 0;
    virtual int inversePosition(RobotModelPtr robot_model,
                                const std::vector<double> &qref,
                                const std::vector<double> &frame,
                                std::vector<double> &qres,
                                bool real = true) = 0;
    virtual int inversePositionAll(RobotModelPtr robot_model,
                                   const std::vector<double> &qref,
                                   const std::vector<double> &frame,
                                   std::vector<std::vector<double>> &qres,
                                   bool real = true) = 0;
    virtual void forwardVelocity(RobotModelPtr robot_model,
                                 const std::vector<double> &q,
                                 const std::vector<double> &qd,
                                 std::vector<double> &twist) = 0;
    virtual int inverseVelocity(RobotModelPtr robot_model,
                                const std::vector<double> &q,
                                const std::vector<double> &twist,
                                std::vector<double> &qdout) = 0;

    /**
     * 校准末端力矩传感器
     */
    virtual bool calibrateTcpForceSensor(
        RobotModelPtr robot_model,
        const std::vector<std::vector<double>> &forces,
        const std::vector<std::vector<double>> &poses,
        std::vector<double> &force_offset, std::vector<double> &com,
        double &mass, std::vector<double> &angle) = 0;

    /**
     * @brief 机械臂运动学标定
     * @param measureData: 测量数据
     * @param jointAngle: 对应的矩阵
     * @param selectionVector: 选择向量
     * @param type: 标定类型
     * @param result: 标定结果(见参考文档)
     * @return: 返回值 < 0, 表示计算失败
     */
    virtual int calibRobotKinematicsPara(
        const std::vector<std::vector<double>> &measureData,
        const std::vector<std::vector<double>> &q,
        const std::vector<bool> &selectionVector, unsigned int type,
        KinCalibResult &result) = 0;

    /**
     * 标定坐标系(需要先标定工具)
     *
     * @param jointAngle: 标定对应的关节角
     * @param type: 标定类型
     * @param result: 标定结果(见参考文档)
     * @return: 返回值 < 0, 表示计算失败
     */
    virtual int calibWorkpieceCoordinatePara(
        const std::vector<std::vector<double>> &q, unsigned int type,
        double *result) = 0;

    /**
     * 碰撞检测
     */
    virtual bool collisionDetect(RobotModelPtr robot_model,
                                 const std::vector<double> &q) = 0;

    // 通过关节电流进行碰撞检测
    virtual bool collisionDetect(const std::vector<double> &q,
                                 const std::vector<double> &curr) = 0;

    // distance为正表示两物体未接触，为负则表示两物体相互嵌入
    virtual double getMinLinkDistance(RobotModelPtr robot_model,
                                      const std::vector<double> &q) = 0;

    virtual std::vector<std::vector<double>> pathMovej(
        RobotModelPtr robot_model, const std::vector<double> &q1, double r1,
        const std::vector<double> &q2, double r2, double d) = 0;

    // 计算关节运动持续时间
    virtual double durationMovej(RobotModelPtr robot_model,
                                 const std::vector<double> &q1,
                                 const std::vector<double> &q2, double v,
                                 double a, double radius_left,
                                 double radius_right, double duration) = 0;

    // type
    // 0-movej and movej
    // 1-movej and movel
    // 2-movel and movej
    // 2-movel and movel
    virtual std::vector<std::vector<double>> pathBlend3Points(
        RobotModelPtr robot_model, int type, const std::vector<double> &q_start,
        const std::vector<double> &q_via, const std::vector<double> &q_to,
        double r, double d) = 0;
};

} // namespace common_interface
} // namespace arcs

#endif // ARAL_EXPORT_KINEMATICS_H
