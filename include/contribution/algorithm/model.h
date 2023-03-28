#ifndef COMMON_INTERFACE_MODEL_H
#define COMMON_INTERFACE_MODEL_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <map>

namespace arcs {
namespace common_interface {
class RobotModel;
using RobotModelPtr = std::shared_ptr<RobotModel>;

/**
 * The RobotModel class
 */
class RobotModel
{
public:
    using VectorDoubleMap =
        std::unordered_map<std::string, std::vector<double>>;

    virtual ~RobotModel() = default;

    /// 获取机器人自由度
    virtual int getDof() const = 0;

    /// 设置机器人类型
    virtual int configRobotModelType(const std::string &type) = 0;

    /// Get name of robot model
    virtual std::string getName() const = 0;

    /// Set gravity vector {0, 0, 9.81}
    virtual void setGravity(const std::vector<double> &gravity) = 0;

    virtual std::vector<double> getGravity() const = 0;
    virtual VectorDoubleMap getDh() const = 0;
    virtual std::vector<double> getLinkMass() const = 0;
    virtual std::vector<double> getJointMotorConstant() const = 0;
    virtual void setJointSoftRanges(const std::vector<double> &lower,
                                    const std::vector<double> &upper) = 0;
    virtual void getJointSoftRanges(std::vector<double> &lower,
                                    std::vector<double> &upper) = 0;

    virtual void getJointRanges(std::vector<double> &lower,
                                std::vector<double> &upper) = 0;

    virtual std::vector<double> getJointMaxVelocities() = 0;
    virtual std::vector<double> getJointMaxAccelerations() = 0;

    virtual std::vector<double> getTcpMaxVelocities() = 0;
    virtual std::vector<double> getTcpMaxAccelerations() = 0;
    virtual double getTcpMaxForce() = 0;

    /**
     * Set TCP offset of the robot
     *
     * @param offset
     */
    virtual int setTcpOffset(const std::vector<double> &offset) = 0;
    virtual std::vector<double> getTcpOffset() = 0;

    /**
     * 返回Payload ID
     * 工具 负载 传感器等
     */
    virtual int attachPayload(double m, const std::vector<double> &com,
                              const std::vector<double> &inertial) = 0;
    /**
     * 移除负载
     */
    virtual void detachPayload(int id) = 0;

    /// 末端力矩传感器安装的位置
    virtual void setEndFtSensorPose(const std::vector<double> &pose) = 0;
    virtual void setBaseFtSensorPose(const std::vector<double> &pose) = 0;

    /// 获取运动学补偿参数
    virtual VectorDoubleMap getKinematicsCompensateParams() = 0;

    /**
     * 设置运动学补偿参数
     */
    virtual bool setKinematicsCompensateParams(
        const VectorDoubleMap &params) = 0;

    /**
     * 设置机械臂真实连杆动力学参数和关节转子惯量(根据辨识结果设置)
     */
    virtual int setRobotDynamicParameter(
        const std::vector<double> &dyn_param) = 0;

    /**
     * 设置电机常数
     */
    virtual int setJointRotorInertia(
        const std::vector<double> &rotor_inertia) = 0;
};

} // namespace common_interface
} // namespace arcs
#endif // ARAL_EXPORT_MODEL_H
