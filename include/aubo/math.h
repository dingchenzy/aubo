#ifndef AUBO_SDK_MATH_INTERFACE_H
#define AUBO_SDK_MATH_INTERFACE_H

#include <vector>
#include <memory>

#include <aubo/type_def.h>
#include <aubo/global_config.h>

namespace arcs {
namespace common_interface {

class ARCS_ABI_EXPORT Math
{
public:
    Math();
    virtual ~Math();

    /**
     * Pose addition
     *
     * Both arguments contain three position parameters (x, y, z) jointly called
     * P, and three rotation parameters (R_x, R_y, R_z) jointly called R. This
     * function calculates the result x_3 as the addition of the given poses as
     * follows:
     *
     * p_3.P = p_1.P + p_2.P
     * p_3.R = p_1.R * p_2.R
     * @param p1: tool pose 1(pose)
     * @param p2: tool pose 2(pose)
     * @return Sum of position parts and product of rotation parts (pose)
     *
     * @code Python函数原型
     * poseAdd(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float]) ->
     * List[float]
     * @endcode
     *
     * @code Lua函数原型
     * poseAdd(p1: table, p2: table) -> table
     * @endcode
     */
    std::vector<double> poseAdd(const std::vector<double> &p1,
                                const std::vector<double> &p2);

    /**
     * Pose subtraction
     *
     * @param p1
     * @param p2
     * @return
     *
     * @code Python函数原型
     * poseSub(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float]) ->
     * List[float]
     * @endcode
     *
     * @code Lua函数原型
     * poseSub(p1: table, p2: table) -> table
     * @endcode
     */
    std::vector<double> poseSub(const std::vector<double> &p1,
                                const std::vector<double> &p2);

    /**
     *
     *
     * @param p1
     * @param p2
     * @param alpha
     * @return
     *
     * @code Python函数原型
     * interpolatePose(self: pyaubo_sdk.Math, arg0: List[float], arg1:
     * List[float], arg2: float) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * interpolatePose(p1: table, p2: table, alpha: number) -> table
     * @endcode
     */
    std::vector<double> interpolatePose(const std::vector<double> &p1,
                                        const std::vector<double> &p2,
                                        double alpha);
    /**
     * Pose transformation
     *
     * The first argument, p_from, is used to transform the second argument,
     * p_from_to, and the result is then returned. This means that the result is
     * the resulting pose, when starting at the coordinate system of p_from, and
     * then in that coordinate system moving p_from_to.
     *
     * This function can be seen in two different views. Either the function
     * transforms, that is translates and rotates, p_from_to by the parameters
     * of p_from. Or the function is used to get the resulting pose, when first
     * making a move of p_from and then from there, a move of p_from_to. If the
     * poses were regarded as transformation matrices, it would look like:
     *
     * ```
     * T_world->to = T_world->from * T_from->to
     * T_x->to = T_x->from * T_from->to
     * ```
     * 已知B相对于A的位姿、C相对于B的位姿，求C相对于A的位姿
     * 第一个参数是B相对于A的位姿，第二个参数是C相对于B的位姿，
     * 返回值是C相对于A的位姿
     *
     * @param pose_from: starting pose (spatial vector)
     * @param pose_from_to: pose change relative to starting pose (spatial
     * vector)
     * @return resulting pose (spatial vector)
     *
     * @code Python函数原型
     * poseTrans(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float]) ->
     * List[float]
     * @endcode
     *
     * @code Lua函数原型
     * poseTrans(pose_from: table, pose_from_to: table) -> table
     * @endcode
     */
    std::vector<double> poseTrans(const std::vector<double> &pose_from,
                                  const std::vector<double> &pose_from_to);

    /**
     *
     * @param pose_from
     * @param pose_to_from
     * @return
     *
     * @code Python函数原型
     * poseTransInv(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float])
     * -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * poseTransInv(pose_from: table, pose_to_from: table) -> table
     * @endcode
     */
    std::vector<double> poseTransInv(const std::vector<double> &pose_from,
                                     const std::vector<double> &pose_to_from);

    /**
     * Get the inverse of a pose
     *
     * @param pose: tool pose (spatial vector)
     * @return inverse tool pose transformation (spatial vector)
     *
     * @code Python函数原型
     * poseInverse(self: pyaubo_sdk.Math, arg0: List[float]) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * poseInverse(pose: table) -> table
     * @endcode
     */
    std::vector<double> poseInverse(const std::vector<double> &pose);

    /**
     * poseDistance
     *
     * @param p1
     * @param p2
     * @return
     */
    double poseDistance(const std::vector<double> &p1,
                        const std::vector<double> &p2);

    double poseAngleDistance(const std::vector<double> &p1,
                             const std::vector<double> &p2);
    bool poseEqual(const std::vector<double> &p1, const std::vector<double> &p2,
                   double eps = 5e-5);

    /**
     *
     * @param F_b_a_old
     * @param V_in_a
     * @param type
     * @return
     *
     * @code Python函数原型
     * transferRefFrame(self: pyaubo_sdk.Math, arg0: List[float], arg1:
     * List[float[3]], arg2: int) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * transferRefFrame(F_b_a_old: table, V_in_a: table, type: number) -> table
     * @endcode
     */
    std::vector<double> transferRefFrame(const std::vector<double> &F_b_a_old,
                                         const Vector3d &V_in_a, int type);

    /**
     * 姿态旋转
     *
     * @param pose
     * @param rotv
     * @return
     *
     * @code Python函数原型
     * poseRotation(self: pyaubo_sdk.Math, arg0: List[float], arg1: List[float])
     * -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * poseRotation(pose: table, rotv: table) -> table
     * @endcode
     */
    std::vector<double> poseRotation(const std::vector<double> &pose,
                                     const std::vector<double> &rotv);

    /**
     * 欧拉角转四元数
     *
     * @param rpy 欧拉角
     * @return 四元数
     *
     * @code Python函数原型
     * rpyToQuaternion(self: pyaubo_sdk.Math, arg0: List[float]) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * rpyToQuaternion(rpy: table) -> table
     * @endcode
     */
    std::vector<double> rpyToQuaternion(const std::vector<double> &rpy);

    /**
     * 四元数转欧拉角
     *
     * @param quat 四元数
     * @return 欧拉角
     *
     * @code Python函数原型
     * quaternionToRpy(self: pyaubo_sdk.Math, arg0: List[float]) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * quaternionToRpy(quat: table) -> table
     * @endcode
     */
    std::vector<double> quaternionToRpy(const std::vector<double> &quat);

    /**
     *
     * @param poses
     * @return
     *
     * @code Python函数原型
     * tcpOffsetIdentify(self: pyaubo_sdk.Math, arg0: List[List[float]]) ->
     * Tuple[List[float], int]
     * @endcode
     *
     * @code Lua函数原型
     * tcpOffsetIdentify(poses: table) -> table
     * @endcode
     */
    ResultWithErrno tcpOffsetIdentify(
        const std::vector<std::vector<double>> &poses);

    /**
     * 三点法标定坐标系
     *
     * @param poses 位姿
     * @param type 类型:\n
     *      0 - oxy 原点 x轴正方向 xy平面（y轴正方向）
     *      1 - oxz 原点 x轴正方向 xz平面（z轴正方向）
     *      2 - oyz 原点 y轴正方向 yz平面（z轴正方向）
     *      3 - oyx 原点 y轴正方向 yx平面（x轴正方向）
     *      4 - ozx 原点 z轴正方向 zx平面（x轴正方向）
     *      5 - ozy 原点 z轴正方向 zy平面（y轴正方向）
     * @return
     */
    ResultWithErrno calibrateCoordinate(
        const std::vector<std::vector<double>> &poses, int type);

protected:
    void *d_;
};
using MathPtr = std::shared_ptr<Math>;

// clang-format off
#define Math_DECLARES                                         \
    _FUNC(Math, 2, poseAdd, p1, p2)                           \
    _FUNC(Math, 2, poseSub, p1, p2)                           \
    _FUNC(Math, 3, interpolatePose, p1, p2, alpha)            \
    _FUNC(Math, 2, poseTrans, pose_from, pose_from_to)        \
    _FUNC(Math, 2, poseTransInv, pose_from, pose_to_from)     \
    _FUNC(Math, 1, poseInverse, pose)                         \
    _FUNC(Math, 2, poseDistance, p1, p2)                      \
    _FUNC(Math, 2, poseAngleDistance, p1, p2)                 \
    _FUNC(Math, 3, poseEqual, p1, p2, eps)                    \
    _FUNC(Math, 3, transferRefFrame, F_b_a_old, V_in_a, type) \
    _FUNC(Math, 2, poseRotation, pose, rotv)                  \
    _FUNC(Math, 1, rpyToQuaternion, rpy)                      \
    _FUNC(Math, 1, quaternionToRpy, quant)                    \
    _FUNC(Math, 1, tcpOffsetIdentify, poses)                  \
    _FUNC(Math, 2, calibrateCoordinate, poses, type)
// clang-format on

} // namespace common_interface
} // namespace arcs
#endif
