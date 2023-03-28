#ifndef COMMON_INTERFACE_OTG_H
#define COMMON_INTERFACE_OTG_H

#include <stddef.h>
#include <vector>
#include <memory>
#include <string.h>

namespace arcs {
namespace common_interface {

class Otg;
using OtgPtr = std::shared_ptr<Otg>;
class RobotModel;
using RobotModelPtr = std::shared_ptr<RobotModel>;

class Otg
{
public:
    enum Mode
    {
        ModePosition = 0,
        ModeVelocity = 1,
    };

    enum Type
    {
        TypeII = 2,
        TypeIII = 3,
        TypeIV = 4,
        TypeIR = 5,
        TypeFollowJoint
    };

    struct Input
    {
        std::vector<double> maxVelocity;
        std::vector<double> maxAcceleration;
        std::vector<double> maxJerk;
        std::vector<double> currentPosition;
        std::vector<double> currentVelocity;
        std::vector<double> currentAcceleration;
        std::vector<double> targetPosition;
        std::vector<double> targetVelocity;
    };

    struct Output
    {
        std::vector<double> posTraj;
        std::vector<double> velTraj;
        std::vector<double> accTraj;
    };

    virtual ~Otg() = default;

    virtual int init(RobotModelPtr robot_model, double cycle_time) = 0;
    virtual int init(int dof, double cycle_time, Otg::Type type,
                     Otg::Mode mode) = 0;

    virtual int setInputParameters(const Input &input,
                                   double move_time = 0) = 0;

    virtual int step(Output &output) = 0;

    virtual int getTrajectoryAtGivenTime(double time, Output &output) = 0;

    virtual double getLeftTime() = 0;
};

} // namespace common_interface
} // namespace arcs
#endif // ARAL_EXPORT_OTG_H
