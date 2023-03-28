#ifndef COMMON_INTERFACE_SENSOR_H
#define COMMON_INTERFACE_SENSOR_H

#include <vector>
#include <string>
#include <memory>

namespace arcs {
namespace common_interface {

/**
 * 传感器设备抽象接口
 */
class SensorHardware
{
public:
    /// 传感器类型
    enum SensorType
    {
        BaseForceSensor,   ///< 底座力矩传感器
        EndForceSensor,    ///< 末端力矩传感器
        JointTorqueSensor, ///< 关节扭矩传感器
        Encoder,           ///< 编码器
        BaseGravity,       ///< 底座重力加速度传感器
    };

    virtual ~SensorHardware() = default;

    /**
     * Get name of manufactory
     *
     * @return Name of manufactory
     */
    virtual std::string getManufactory() const = 0;

    /**
     * 获取设备型号名称
     */
    virtual std::string getModelType() const = 0;

    /**
     * 使能
     */
    virtual void enable() = 0;

    /**
     * 禁用
     */
    virtual void disable() = 0;

    /**
     * 周期性更新传感器读数
     */
    virtual void update(double elapse) = 0;

    /**
     * 获取传感器类型
     */
    virtual SensorType getType() const = 0;

    /**
     * 通道数
     */
    virtual int getDimension() const = 0;

    /**
     * If every channel vaildated
     *
     * @return
     */
    virtual std::vector<bool> isValid() const = 0;

    /**
     * Sensor readings
     *
     * @return
     */
    virtual std::vector<double> getData() const = 0;

    /**
     * 获取传感器量程
     *
     * @return
     */
    virtual std::vector<double> getScale() const = 0;

    /**
     * 获取传感器精度
     *
     * @return
     */
    virtual std::vector<double> getPrecision() const = 0;

    /**
     * 获取传感器错误
     *
     * @return
     */
    virtual int getError() = 0;
};

using SensorHardwarePtr = std::shared_ptr<SensorHardware>;

} // namespace common_interface
} // namespace arcs
#endif // COMMON_INTERFACE_SENSOR_H
