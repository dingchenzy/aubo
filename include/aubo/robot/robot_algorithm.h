#ifndef AUBO_SDK_ROBOT_ALGORITHM_INTERFACE_H
#define AUBO_SDK_ROBOT_ALGORITHM_INTERFACE_H

#include <string>
#include <vector>
#include <memory>
#include <functional>

#include <aubo/global_config.h>
#include <aubo/type_def.h>

namespace arcs {
namespace common_interface {

/**
 * 机器人算法相关的对外接口
 */
class ARCS_ABI_EXPORT RobotAlgorithm
{
public:
    RobotAlgorithm();
    virtual ~RobotAlgorithm();

    /**
     * 力传感器标定算法(三点标定法)
     *
     * @param force
     * @param q
     * @return
     *
     * @code Python函数原型
     * calibrateTcpForceSensor(self: pyaubo_sdk.RobotAlgorithm, arg0:
     * List[List[float]], arg1: List[List[float]]) -> Tuple[List[float],
     * List[float], float, List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * calibrateTcpForceSensor(force: table, q: table) -> table
     * @endcode
     */
    ForceSensorCalibResult calibrateTcpForceSensor(
        const std::vector<std::vector<double>> &forces,
        const std::vector<std::vector<double>> &poses);

    /**
     * 负载辨识算法接口
     * 输入关节角度和(工具端)力矩传感器读数，输出质量，质心偏移等等
     *
     * @param q 关节角度集合
     * @param forces 对应的末端力/力矩集合
     * @return 辨识的结果
     *
     * @code Python函数原型
     * payloadIdentify(self: pyaubo_sdk.RobotAlgorithm, arg0: List[List[float]],
     * arg1: List[List[float]]) -> Tuple[List[float], List[float], float,
     * List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * payloadIdentify(q: table, forces: table) -> table
     * @endcode
     */
    ForceSensorCalibResult payloadIdentify(
        const std::vector<std::vector<double>> &q,
        const std::vector<std::vector<double>> &forces);

    /**
     * 关节摩擦力模型辨识算法接口
     *
     * @param q
     * @param qd
     * @param qdd
     * @param temp
     * @return
     *
     * @code Python函数原型
     * frictionModelIdentify(self: pyaubo_sdk.RobotAlgorithm, arg0:
     * List[List[float]], arg1: List[List[float]], arg2: List[List[float]],
     * arg3: List[List[float]]) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * frictionModelIdentify(q: table, qd: table, qdd: table, temp: table) ->
     * boolean
     * @endcode
     */
    bool frictionModelIdentify(const std::vector<std::vector<double>> &q,
                               const std::vector<std::vector<double>> &qd,
                               const std::vector<std::vector<double>> &qdd,
                               const std::vector<std::vector<double>> &temp);

    /**
     * 工件坐标系标定算法接口(需要在调用之前正确的设置机器人的TCP偏移)
     * 输入多组关节角度和标定类型，输出工件坐标系位姿(相对于机器人基坐标系)
     *
     * @param q 关节角度
     * @param type 标定类型
     * @return 计算结果(工件坐标系位姿)以及错误代码
     *
     * @code Python函数原型
     * calibWorkpieceCoordinatePara(self: pyaubo_sdk.RobotAlgorithm, arg0:
     * List[List[float]], arg1: int) -> Tuple[List[float], int]
     * @endcode
     *
     * @code Lua函数原型
     * calibWorkpieceCoordinatePara(q: table, type: number) -> table
     * @endcode
     */
    ResultWithErrno calibWorkpieceCoordinatePara(
        const std::vector<std::vector<double>> &q, int type);

    /**
     * 动力学正解
     *
     * @param q 关节角度
     * @param torqs
     * @return 计算结果以及错误代码
     *
     * @code Python函数原型
     * forwardDynamics(self: pyaubo_sdk.RobotAlgorithm, arg0: List[float], arg1:
     * List[float]) -> Tuple[List[float], int]
     * @endcode
     *
     * @code Lua函数原型
     * forwardDynamics(q: table, torqs: table) -> number
     * @endcode
     */
    ResultWithErrno forwardDynamics(const std::vector<double> &q,
                                    const std::vector<double> &torqs);

    /**
     * 运动学正解
     * 输入关节角度，输出TCP位姿
     *
     * @param q 关节角度
     * @return 计算结果(TCP位姿)以及错误代码
     *
     * @code Python函数原型
     * forwardKinematics(self: pyaubo_sdk.RobotAlgorithm, arg0: List[float]) ->
     * Tuple[List[float], int]
     * @endcode
     *
     * @code Lua函数原型
     * forwardKinematics(q: table) -> table
     * @endcode
     */
    ResultWithErrno forwardKinematics(const std::vector<double> &q);

    ResultWithErrno forwardToolKinematics(const std::vector<double> &q);

    /**
     * 运动学逆解
     * 输入TCP位姿和参考关节角度，输出关节角度
     *
     * @param qnear 参考关节角度
     * @param pose TCP位姿
     * @return 计算结果(关节角)以及错误代码
     *
     * @code Python函数原型
     * inverseKinematics(self: pyaubo_sdk.RobotAlgorithm, arg0: List[float],
     * arg1: List[float]) -> Tuple[List[float], int]
     * @endcode
     *
     * @code Lua函数原型
     * inverseKinematics(qnear: table, pose: table) -> table
     * @endcode
     */
    ResultWithErrno inverseKinematics(const std::vector<double> &qnear,
                                      const std::vector<double> &pose);

    /**
     * 求出所有的逆解
     */
    ResultWithErrno1 inverseKinematicsAll(const std::vector<double> &pose);

    /**
     * 求解movej之间的轨迹点
     *
     * @param q1 movej的起点
     * @param r1 在q1处的交融半径
     * @param q2 movej的终点
     * @param r2 在q2处的交融半径
     * @param d 采样距离
     * @return q1~q2 之间不包含交融段的笛卡尔空间离散轨迹点(x,y,z)集合
     *
     * @code Python函数原型
     * pathMovej(self: pyaubo_sdk.RobotAlgorithm, arg0: List[float], arg1:
     * float, arg2: List[float], arg3: float, arg4: float) -> List[List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * pathMovej(q1: table, r1: number, q2: table, r2: number, d: number) ->
     * table
     * @endcode
     */
    std::vector<std::vector<double>> pathMovej(const std::vector<double> &q1,
                                               double r1,
                                               const std::vector<double> &q2,
                                               double r2, double d);

    /**
     * 求解交融的轨迹点
     *
     * @param type
     * 0-movej and movej
     * 1-movej and movel
     * 2-movel and movej
     * 2-movel and movel
     * @param q_start 交融前路径的起点
     * @param q_via 在q1处的交融半径
     * @param q_to 交融后路径的终点
     * @param r 在q_via处的交融半径
     * @param d 采样距离
     * @return q_via处的交融段笛卡尔空间离散轨迹点(x,y,z)集合
     *
     * @code Python函数原型
     * pathBlend3Points(self: pyaubo_sdk.RobotAlgorithm, arg0: int, arg1:
     * List[float], arg2: List[float], arg3: List[float], arg4: float, arg5:
     * float) -> List[List[float]]
     * @endcode
     *
     * @code Lua函数原型
     * pathBlend3Points(type: number, q_start: table, q_via: table, q_to: table,
     * r: number, d: number) -> table
     * @endcode
     */
    std::vector<std::vector<double>> pathBlend3Points(
        int type, const std::vector<double> &q_start,
        const std::vector<double> &q_via, const std::vector<double> &q_to,
        double r, double d);

protected:
    void *d_;
};
using RobotAlgorithmPtr = std::shared_ptr<RobotAlgorithm>;

// clang-format off
#define RobotAlgorithm_DECLARES                                        \
    _FUNC(RobotAlgorithm, 2, calibrateTcpForceSensor, forces, poses)   \
    _FUNC(RobotAlgorithm, 2, payloadIdentify, q, forces)               \
    _FUNC(RobotAlgorithm, 4, frictionModelIdentify, q, qd, qdd, temp)  \
    _FUNC(RobotAlgorithm, 2, calibWorkpieceCoordinatePara, q, type)    \
    _FUNC(RobotAlgorithm, 2, forwardDynamics, q, torqs)                \
    _FUNC(RobotAlgorithm, 1, forwardKinematics, q)                     \
    _FUNC(RobotAlgorithm, 1, forwardToolKinematics, q)                 \
    _FUNC(RobotAlgorithm, 2, inverseKinematics, qnear, pose)           \
    _FUNC(RobotAlgorithm, 1, inverseKinematicsAll, pose)               \
    _FUNC(RobotAlgorithm, 5, pathMovej, q1, r1, q2, r2, override)      \
    _FUNC(RobotAlgorithm, 6, pathBlend3Points, type, q_start, q_via, q_to, r, d)

// clang-format on

} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_ROBOT_ALGORITHM_INTERFACE_H
