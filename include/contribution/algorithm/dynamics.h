#ifndef COMMON_INTERFACE_DYNAMICS_H
#define COMMON_INTERFACE_DYNAMICS_H

#include <vector>
#include <memory>
#include <sstream>
#include <contribution/logging.h>

namespace arcs {
namespace common_interface {

class DynamicsSolver;
using DynamicsSolverPtr = std::shared_ptr<DynamicsSolver>;
class RobotModel;
using RobotModelPtr = std::shared_ptr<RobotModel>;

template <typename T>
inline std::string toString(const std::vector<T> &list)
{
    std::stringstream os;
    for (size_t i = 0; i < list.size(); i++) {
        os << list.at(i);
        if (i != (list.size() - 1)) {
            os << ",";
        }
    }
    return os.str();
}

/**
 * The DynamicsSolver class
 */
class DynamicsSolver
{
public:
    // 导纳控制输入参数
    struct Input
    {
        std::vector<double> q;      ///< 当前实际TCP位置
        std::vector<double> qd;     ///< 当前实际TCP速度
        std::vector<double> qdd;    ///< 当前实际TCP加速度
        std::vector<double> qref;   ///< 参考TCP角度
        std::vector<double> qdref;  ///< 参考TCP速度
        std::vector<double> qddref; ///< 参考TCP加速度
        std::vector<double> xref;   ///< 参考TCP角度
        std::vector<double> xdref;  ///< 参考TCP速度
        std::vector<double> xddref; ///< 参考TCP加速度
        std::vector<double> wrench; ///< 实际末端力矩传感器读数
        std::vector<double> frame;  ///< 力控参考坐标系
        std::vector<bool> select;   ///< 力控选择向量
        std::vector<double> target_wrench; ///< 目标力矩
        std::vector<double> spd_limit;     ///< 速度限制
        std::vector<double> k;             ///< k
        std::vector<double> d;             ///< d
        std::vector<double> m;             ///< m
        std::vector<double> p; ///< 用于调节达到目标力的速度
        std::vector<double> x_last_error; ///< 上一周期笛卡尔位置误差
        std::vector<double> xd_last_error; ///< 上一周期笛卡尔速度误差
        std::vector<double> q_last_error; ///< 上一周期关节空间位置误差
        std::vector<double> qd_last_error; ///< 上一周期关节空间速度误差
        std::vector<double> qdd_last_error; ///< 上一周期关节空间加速度误差
        std::vector<double> last_err_wrench;          ///< 上一周期力误差
        std::vector<double> last_integral_err_wrench; ///<上一周期力误差积分
        std::vector<double> min_force; ///< 末端力低于min_force按零处理
        std::vector<double> max_force; ///< 末端力高于max_force,
                                       ///< 末端力按照max_force处理
    };

    inline std::string toString(const Input &input)
    {
        std::stringstream os;
        os << "q {" << common_interface::toString(input.q) << "}\n";
        os << "qd {" << common_interface::toString(input.qd) << "}\n";
        os << "qdd {" << common_interface::toString(input.qdd) << "}\n";
        os << "qref {" << common_interface::toString(input.qref) << "}\n";
        os << "qdref {" << common_interface::toString(input.qdref) << "}\n";
        os << "qddref {" << common_interface::toString(input.qddref) << "}\n";
        os << "xref {" << common_interface::toString(input.xref) << "}\n";
        os << "xdref {" << common_interface::toString(input.xdref) << "}\n";
        os << "xddref {" << common_interface::toString(input.xddref) << "}\n";
        os << "wrench {" << common_interface::toString(input.wrench) << "}\n";
        os << "frame {" << common_interface::toString(input.frame) << "}\n";
        os << "select {" << common_interface::toString(input.select) << "}\n";
        os << "target_wrench {"
           << common_interface::toString(input.target_wrench) << "}\n";
        os << "spd_limit {" << common_interface::toString(input.spd_limit)
           << "}\n";
        os << "k {" << common_interface::toString(input.k) << "}\n";
        os << "d {" << common_interface::toString(input.d) << "}\n";
        os << "m {" << common_interface::toString(input.m) << "}\n";
        return os.str();
    }

    // 导纳控制输出参数
    struct Output
    {
        std::vector<double> q;                   /// TCP位置
        std::vector<double> qd;                  /// TCP速度
        std::vector<double> qdd;                 /// TCP加速度
        std::vector<double> x;                   /// TCP位置
        std::vector<double> xd;                  /// TCP速度
        std::vector<double> xdd;                 /// TCP加速度
        std::vector<double> wrench;              /// TCP加速度
        std::vector<double> x_error;             /// 笛卡尔位置误差
        std::vector<double> xd_error;            /// 笛卡尔速度误差
        std::vector<double> q_error;             /// 关节空间位置误差
        std::vector<double> qd_error;            /// 关节空间速度误差
        std::vector<double> qdd_error;           /// 关节空间加速度误差
        std::vector<double> err_wrench;          /// 力误差
        std::vector<double> integral_err_wrench; ///力误差积分
    };

    inline std::string toString(const Output &output)
    {
        std::stringstream os;
        os << "q {" << common_interface::toString(output.q) << "}\n";
        os << "qd {" << common_interface::toString(output.qd) << "}\n";
        os << "qdd {" << common_interface::toString(output.qdd) << "}\n";
        os << "x {" << common_interface::toString(output.x) << "}\n";
        os << "xd {" << common_interface::toString(output.xd) << "}\n";
        os << "xdd {" << common_interface::toString(output.xdd) << "}\n";
        os << "wrench {" << common_interface::toString(output.wrench) << "}\n";
        return os.str();
    }

    //末端传感器辨识结果
    struct FtSensorCalibResult
    {
        std::vector<double> offset; /// 传感器偏置
        std::vector<double> com;    /// 工具在传感器坐标系下的质心
        double mass;                /// 工具质量
        double angle[2]; /// 机械臂底座的安装角度（一般不用该参数）
    };

    inline std::string toString(const FtSensorCalibResult &result)
    {
        std::stringstream os;
        os << "offset {" << common_interface::toString(result.offset) << "}\n";
        os << "com {" << common_interface::toString(result.com) << "}\n";
        os << "mass " << result.mass << "\n";
        os << "angle {" << result.angle[0] << "," << result.angle[1] << "}\n";
        return os.str();
    }

    virtual ~DynamicsSolver() = default;

    /** 动力学正解
     *
     * @param q Joint positions
     */
    virtual std::vector<double> forward(
        RobotModelPtr robot_model, const std::vector<double> &q,
        const std::vector<double> &qd, const std::vector<double> &torque,
        const std::vector<std::vector<double>> &ft_ext, bool real_fd) = 0;

    /**
     * 动力学逆解
     *
     * @return
     */
    virtual std::vector<double> inverse(RobotModelPtr robot_model,
                                        const std::vector<double> &q,
                                        const std::vector<double> &qd,
                                        const std::vector<double> &qdd,
                                        bool real_id) = 0;

    /**
     * 计算科氏力
     *
     * @return
     */
    virtual std::vector<double> calCoriolisTorque(RobotModelPtr robot_model,
                                                  const std::vector<double> &q,
                                                  const std::vector<double> &qd,
                                                  bool real_id) = 0;

    /**
     * 计算拖动示教阻抗补偿项
     *
     * @param robot_model 机器人模型
     * @param q 当前关节角度
     * @param qd 当前关节速度
     * @param damp 正常拖动下的关节阻尼(限制速度)
     * @param limit_damp 速度限制阻尼
     * @param limit_stiff 位置限制刚度
     * @param stiff_interval_width 刚度调节区间宽度
     * @return 需要补偿力矩
     */
    virtual std::vector<double> calJointImpendance(
        RobotModelPtr robot_model, const std::vector<double> &q,
        const std::vector<double> &qd, const std::vector<double> &damp,
        const std::vector<double> &limit_damp,
        const std::vector<double> &limit_stiff,
        const std::vector<double> &stiff_interval_width) = 0;

    /**
     * 根据末端6维力/力矩传感器的信息标定工具和传感器的属性
     *
     * @param q:
     * 输入三种不同构型对应的机器人关节角，三种构型的姿态差异越大越好
     * @param measurement：在三种构型下测得的传感器的原始裸数据
     * @param result: 1) offset: 传感器的零点偏置值
     *                2) com: 工具在传感器坐标系下的质心位置
     *                3) mass: 工具的质量
     *                4) angle: 机器人底座在世界坐标系下的姿态角。
     * @return
     */
    virtual int calibToolAndSensor(const std::vector<std::vector<double>> q,
                                   const std::vector<std::vector<double>> torqs,
                                   FtSensorCalibResult &result) = 0;

    /**
     * calibDynamicPara: cali dynamic para;
     *
     * @param[in] q
     * @param[in] jointVelocity
     * @param[in] jointAcceleration
     * @param[in] measuredata
     * @param[in] type: internal_sdp = 0, internal_ols = 1, external_ols = 2,
     * @param[in] paraEst
     * @return: 返回值 < 0, 表示计算失败
     */
    virtual int calibDynamicPara(const std::vector<std::vector<double>> &q,
                                 const std::vector<std::vector<double>> &qref,
                                 unsigned int type,
                                 std::vector<double> &paraEst) = 0;

    /**
     * @brief fcEnable
     * @param robot_model
     * @return
     */
    virtual int fcEnable(RobotModelPtr robot_model) = 0;

    /**
     * @brief fcDisable
     * @param robot_model
     * @return
     */
    virtual int fcDisable(RobotModelPtr robot_model) = 0;

    /**
     * @brief admitance
     * @param robot_model
     * @param input
     * @param output
     * @param period
     * @return
     */
    virtual int admitance(RobotModelPtr robot_model, const Input &input,
                          Output &output, double period) = 0;

    /**
     * @brief jointAdmitance 关节空间导纳
     * @param robot_model
     * @param input
     * @param output
     * @return
     */
    virtual int jointAdmitance(RobotModelPtr robot_model, const Input &input,
                               Output &output) = 0;

    /**
     * @brief detectRobotCollision
     * @param robot_model
     * @param q 当前位置
     * @param qd 当前速度
     * @param qdd 当前加速度
     * @param torq 关节估计力矩(去除摩擦力), 单位Nm
     * @param collision_threshold 碰撞阈值, 单位Nm
     * @return 各关节碰撞标志, true碰撞, false无碰撞
     */
    virtual std::vector<bool> detectRobotCollision(
        RobotModelPtr robot_model, const std::vector<double> &q,
        const std::vector<double> &qd, const std::vector<double> &qdd,
        const std::vector<double> &torq,
        const std::vector<double> &collision_threshold) = 0;
};

} // namespace common_interface
} // namespace arcs

#endif // ARAL_EXPORT_DYNAMICS_H
