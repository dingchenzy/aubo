#ifndef COMMON_INTERFACE_AGV_H
#define COMMON_INTERFACE_AGV_H

#include <string>
#include <vector>

namespace arcs {
namespace common_interface {

struct AgvMap
{
};

class AgvHardware
{
public:
    virtual ~AgvHardware() = default;

    std::string getName();

    double getMileAge();
    double getTodayMileAge();
    double getAge();
    double getControllerTemperature();
    double getVolatage();

    double getBatteryVoltage();

    std::vector<double> getLaser();

    std::vector<double> getPostion();
    std::vector<double> getSpeed();

    std::vector<double> getWheelPostion();
    std::vector<double> getWheelSpeed();

    std::vector<std::string> getMapNames();
    AgvMap getMap(const std::string &name);
    int setMap(const std::string &name, AgvMap map);

    /// 建图
    int slamStart();
    int slamStop();

    int powerOn();
    int enableMotor();

    /// 重定位
    int relocation();

    int moveTo(const std::vector<double> &pos);
    int speedTo(const std::vector<double> &spd);

    int moveStop();
};

} // namespace common_interface
} // namespace arcs
#endif
