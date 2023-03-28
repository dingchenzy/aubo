#ifndef COMMON_INTERFACE_ALGORITHM_FACTORY_H
#define COMMON_INTERFACE_ALGORITHM_FACTORY_H

#include <string>

#include <aubo/type_def.h>
#include "contribution/algorithm/dynamics.h"
#include "contribution/algorithm/kinematics.h"
#include "contribution/algorithm/model.h"
#include "contribution/algorithm/otg.h"
#include "contribution/algorithm/tp.h"
#include "contribution/device/io.h"
#include "contribution/device/robot.h"
#include "contribution/device/sensor.h"
#include "contribution/extension.h"

namespace arcs {
namespace common_interface {

/**
 * The AbstractFactory class
 *
 * @tparam T
 */
template <typename T>
class AbstractFactory
{
public:
    virtual ~AbstractFactory() = default;

    /**
     * Create Service instance
     *
     * @param log_handler
     * @param config toml format configure string
     * @return
     */
    virtual std::shared_ptr<T> create(LogHandler &&log_handler,
                                      const std::string &config = "") = 0;
};

template <typename T>
struct ConfigNameTraits
{
};

template <typename T>
inline constexpr const char *getConfigName()
{
    return ConfigNameTraits<T>::name;
}

#define DECLARE_SERVICE(cls, config_name)                  \
    using cls##Factory = AbstractFactory<cls>;             \
    using cls##FactoryPtr = std::shared_ptr<cls##Factory>; \
    template <>                                            \
    struct ConfigNameTraits<cls>                           \
    {                                                      \
        static constexpr const char *name = config_name;   \
    };

/// Create RobotModel
DECLARE_SERVICE(RobotModel, "robot_model_alg")

/// Create KinematicsSolver
DECLARE_SERVICE(KinematicsSolver, "kinematics_alg")

/// Create DynamicsSolver
DECLARE_SERVICE(DynamicsSolver, "dynamics_alg")

/// Create TrajectoryPlan
DECLARE_SERVICE(TrajectoryPlan, "trajectory_plan_alg")

/// Create Otg
DECLARE_SERVICE(Otg, "otg_alg")

class AlgorithmFactory
{
public:
    virtual ~AlgorithmFactory() = default;

    virtual int setUp(LogHandler &&log_handler,
                      const std::string &config = "") = 0;

    virtual RobotModelPtr createRobotModel(const std::string &robot_type) = 0;
    virtual KinematicsSolverPtr createKinematicsSolver() = 0;
    virtual DynamicsSolverPtr createDynamicsSolver() = 0;
    virtual TrajectoryPlanPtr createTrajectoryPlan() = 0;
    virtual OtgPtr createOtg() = 0;
};
using AlgorithmFactoryPtr = std::shared_ptr<AlgorithmFactory>;
template <>
struct ConfigNameTraits<AlgorithmFactory>
{
    static constexpr const char *name = "algorithm";
};

/// Create IoHardware
using IoFactory = AbstractFactory<IoHardware>;
using IoFactoryPtr = std::shared_ptr<IoFactory>;

/// Create RobotHardware
using RobotFactory = AbstractFactory<RobotHardware>;
using RobotFactoryPtr = std::shared_ptr<RobotFactory>;

/// Create SensorHardware
using SensorFacroty = AbstractFactory<SensorHardware>;
using SensorFacrotyPtr = std::shared_ptr<SensorFacroty>;

} // namespace common_interface
} // namespace arcs
#endif // COMMON_INTERFACE_ALGORITHM_FACTORY_H
