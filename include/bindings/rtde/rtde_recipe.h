#ifndef COMMON_INTERFACE_RTDE_RECIPE_H
#define COMMON_INTERFACE_RTDE_RECIPE_H

#include <stdint.h>
#include <vector>
#include <string>

#include <aubo/aubo_api.h>

namespace arcs {
namespace common_interface {

/// RRII(name, enum_number, type, document)

#define ROBOT_RTDE_INPUT_MAP(R, OFFSET)                                        \
    RRII(R##_speed_slider_mask, OFFSET + 0, double,                            \
         "0 = don't change speed slider with this input \n"                    \
         "1 = use speed_slider_fraction to set speed slider value")            \
    RRII(R##_speed_slider_fraction, OFFSET + 1, double,                        \
         "new speed slider value ")                                            \
    RRII(R##_standard_digital_output_mask, OFFSET + 2, uint32_t,               \
         "Standard digital outputs ")                                          \
    RRII(R##_configurable_digital_output_mask, OFFSET + 3, uint32_t,           \
         "Configurable digital outputs")                                       \
    RRII(R##_standard_digital_output, OFFSET + 4, uint32_t,                    \
         "Standard digital outputs ")                                          \
    RRII(R##_configurable_digital_output, OFFSET + 5, uint32_t,                \
         "Configurable digital outputs")                                       \
    RRII(R##_tool_digital_output, OFFSET + 6, uint32_t,                        \
         "Tool digital outputs \n"                                             \
         "Bits 0-1: output state, remaining bits are reserved for future use") \
    RRII(R##_standard_analog_output_type, OFFSET + 7, std::vector<int>,        \
         "Output domain {0=current[A], 1=voltage[V]} \n "                      \
         "Bits 0-1: standard_analog_output_0 | standard_analog_output_1")      \
    RRII(R##_standard_analog_output_mask, OFFSET + 8, uint32_t,                \
         "Standard analog output 0 (ratio) [0..1] ")                           \
    RRII(R##_standard_analog_output, OFFSET + 9, std::vector<double>,          \
         "Standard analog output 1 (ratio) [0..1] ")                           \
    RRII(R##_debug, OFFSET + 10, uint32_t, "Debug for internal use ")          \
    RRII(R##_rtde_input_max, OFFSET + 11, int, "")

#define ROBOT_RTDE_OUTPUT_MAP(R, OFFSET)                                      \
    RRII(R##_message, OFFSET + 0, RobotMsg, "Robot message from controller")  \
    RRII(R##_target_q, OFFSET + 1, std::vector<double>,                       \
         "Target joint positions")                                            \
    RRII(R##_target_qd, OFFSET + 2, std::vector<double>,                      \
         "Target joint velocities")                                           \
    RRII(R##_target_qdd, OFFSET + 3, std::vector<double>,                     \
         "Target joint accelerations")                                        \
    RRII(R##_target_current, OFFSET + 4, std::vector<double>,                 \
         "Target joint currents")                                             \
    RRII(R##_target_moment, OFFSET + 5, std::vector<double>,                  \
         "Target joint moments (torques)")                                    \
    RRII(R##_actual_q, OFFSET + 6, std::vector<double>,                       \
         "Actual joint positions")                                            \
    RRII(R##_actual_qd, OFFSET + 7, std::vector<double>,                      \
         "Actual joint velocities")                                           \
    RRII(R##_actual_current, OFFSET + 8, std::vector<double>,                 \
         "Actual joint currents")                                             \
    RRII(R##_joint_control_output, OFFSET + 9, std::vector<double>,           \
         "Joint control currents")                                            \
    RRII(R##_joint_temperatures, OFFSET + 10, std::vector<double>,            \
         "Temperature of each joint in degrees Celsius")                      \
    RRII(R##_actual_joint_voltage, OFFSET + 11, std::vector<double>,          \
         "Actual joint voltages")                                             \
    RRII(R##_joint_mode, OFFSET + 12, std::vector<JointStateType>,            \
         "Joint control modes Please see Remote Control Via TCP/IP - 16496")  \
    RRII(R##_actual_execution_time, OFFSET + 13, double,                      \
         "Controller real-time thread execution time")                        \
    RRII(R##_robot_mode, OFFSET + 14, RobotModeType,                          \
         "Robot mode Please see Remote Control Via TCP/IP - 16496")           \
    RRII(R##_safety_mode, OFFSET + 15, SafetyModeType,                        \
         "Safety mode Please see Remote Control Via TCP/IP - 16496")          \
    RRII(R##_safety_status, OFFSET + 16, unknown, "Safety ststus")            \
    RRII(R##_robot_status_bits, OFFSET + 17, unknown,                         \
         "Bits 0-3: Is power on | Is program running | Is"                    \
         "teach button pressed | Is power button pressed")                    \
    RRII(R##_safety_status_bits, OFFSET + 18, unknown,                        \
         "Bits 0-10: Is normal mode | Is reduced mode | | Is protective"      \
         "stopped | Is recovery mode | Is safeguard stopped | Is system"      \
         "emergency stopped | Is robot emergency stopped | Is emergency"      \
         "stopped | Is violation | Is fault | Is stopped due to safety")      \
    RRII(R##_speed_scaling, OFFSET + 19, double,                              \
         "Speed scaling of the trajectory limiter")                           \
    RRII(R##_target_speed_fraction, OFFSET + 20, double,                      \
         "Target speed fraction")                                             \
    RRII(R##_actual_TCP_pose, OFFSET + 21, std::vector<double>,               \
         "Actual Cartesian coordinates of the tool:"                          \
         "(x,y,z,rx,ry,rz), where rx, ry and rz is a rotation"                \
         "vector representation of the tool orientation")                     \
    RRII(R##_actual_TCP_speed, OFFSET + 22, std::vector<double>,              \
         "Actual speed of the tool given in Cartesian coordinates")           \
    RRII(R##_actual_TCP_force, OFFSET + 23, std::vector<double>,              \
         "Generalized forces in the TCP")                                     \
    RRII(R##_target_TCP_pose, OFFSET + 24, std::vector<double>,               \
         "Target Cartesian coordinates of the tool:"                          \
         "(x,y,z,rx,ry,rz), where rx, ry and rz is a rotation"                \
         "vector representation of the tool orientation")                     \
    RRII(R##_target_TCP_speed, OFFSET + 25, std::vector<double>,              \
         "Target speed of the tool given in Cartesian coordinates")           \
    RRII(R##_elbow_position, OFFSET + 26, std::vector<double>,                \
         "Position of robot elbow in Cartesian Base Coordinates")             \
    RRII(R##_elbow_velocity, OFFSET + 27, std::vector<double>,                \
         "Velocity of robot elbow in Cartesian Base Coordinates")             \
    RRII(R##_actual_momentum, OFFSET + 28, std::vector<double>,               \
         "Norm of Cartesian linear momentum")                                 \
    RRII(R##_tcp_force_scalar, OFFSET + 29, std::vector<double>,              \
         "TCP force scalar [N]")                                              \
    RRII(R##_actual_main_voltage, OFFSET + 31, unknown,                       \
         "Safety Control Board: Main voltage")                                \
    RRII(R##_actual_robot_voltage, OFFSET + 32, unknown,                      \
         "Safety Control Board: Robot voltage (48V)")                         \
    RRII(R##_actual_robot_current, OFFSET + 33, unknown,                      \
         "Safety Control Board: Robot current")                               \
    RRII(R##_operationalModeSelectorInput, OFFSET + 40, unknown, "")          \
    RRII(R##_threePositionEnablingDeviceInput, OFFSET + 41, unknown, "")      \
    RRII(R##_masterboard_temperature, OFFSET + 42, unknown, "")               \
    RRII(R##_standard_digital_input_bits, OFFSET + 43, uint64_t,              \
         "Current state of the digital inputs. 0-7:"                          \
         "Standard, 8-15: Configurable, 16-17: Tool")                         \
    RRII(R##_tool_digital_input_bits, OFFSET + 44, uint64_t,                  \
         "Current state of the digital inputs. 0-7:"                          \
         "Standard, 8-15: Configurable, 16-17: Tool")                         \
    RRII(R##_configurable_digital_input_bits, OFFSET + 45, uint64_t,          \
         "Current state of the digital inputs. 0-7:"                          \
         "Standard, 8-15: Configurable, 16-17: Tool")                         \
    RRII(R##_link_digital_input_bits, OFFSET + 47, uint64_t,                  \
         "Current state of the digital inputs. 0-7:"                          \
         "Standard, 8-15: Configurable, 16-17: Tool")                         \
    RRII(R##_standard_digital_output_bits, OFFSET + 48, uint64_t,             \
         "Current state of the digital outputs. 0-7: Standard, 8-15:"         \
         "Configurable, 16-17: Tool")                                         \
    RRII(R##_tool_digital_output_bits, OFFSET + 49, uint64_t,                 \
         "Current state of the digital outputs. 0-7: Standard, 8-15:"         \
         "Configurable, 16-17: Tool")                                         \
    RRII(R##_configurable_digital_output_bits, OFFSET + 50, uint64_t,         \
         "Current state of the digital outputs. 0-7: Standard, 8-15:"         \
         "Configurable, 16-17: Tool")                                         \
    RRII(R##_link_digital_output_bits, OFFSET + 52, uint64_t,                 \
         "Current state of the digital outputs. 0-7: Standard, 8-15:"         \
         "Configurable, 16-17: Tool")                                         \
    RRII(R##_standard_analog_input_values, OFFSET + 53, std::vector<double>,  \
         "")                                                                  \
    RRII(R##_tool_analog_input_values, OFFSET + 54, std::vector<double>, "")  \
    RRII(R##_standard_analog_output_values, OFFSET + 55, std::vector<double>, \
         "")                                                                  \
    RRII(R##_tool_analog_output_values, OFFSET + 56, std::vector<double>, "") \
    RRII(R##_master_io_current, OFFSET + 60, unknown, "I/O current [A]")      \
    RRII(R##_euromap67_input_bits, OFFSET + 61, unknown,                      \
         "Euromap67 input bits")                                              \
    RRII(R##_euromap67_output_bits, OFFSET + 62, unknown,                     \
         "Euromap67 output bits")                                             \
    RRII(R##_euromap67_24V_voltage, OFFSET + 63, unknown,                     \
         "Euromap 24V voltage [V]")                                           \
    RRII(R##_euromap67_24V_current, OFFSET + 64, unknown,                     \
         "Euromap 24V current [A]")                                           \
    RRII(R##_tool_mode, OFFSET + 65, unknown,                                 \
         "Tool mode Please see Remote Control Via TCP/IP - 16496")            \
    RRII(R##_tool_output_mode, OFFSET + 66, unknown,                          \
         "The current output mode")                                           \
    RRII(R##_tool_output_voltage, OFFSET + 67, unknown,                       \
         "Tool output voltage [V]")                                           \
    RRII(R##_tool_output_current, OFFSET + 68, unknown, "Tool current [A]")   \
    RRII(R##_tool_voltage_48V, OFFSET + 69, unknown, "")                      \
    RRII(R##_tool_current, OFFSET + 70, unknown, "")                          \
    RRII(R##_tool_temperature, OFFSET + 71, unknown,                          \
         "Tool temperature in degrees Celsius")                               \
    RRII(R##_actual_tool_accelerometer, OFFSET + 72, unknown,                 \
         "Tool x, y and z accelerometer values")                              \
    RRII(R##_motion_progress, OFFSET + 73, unknown,                           \
         "Trajectory running progress")                                       \
    RRII(R##_actual_qdd, OFFSET + 74, unknown, "Actual joint accelerations")  \
    RRII(R##_rtde_output_max, OFFSET + 75, int, "")

#define RTDE_INPUT_MAP                                                         \
    RRII(set_recipe, 100, RtdeRecipe, "Set ")                                  \
    RRII(input_bit_registers0_to_31, 101, int,                                 \
         "General purpose bits "                                               \
         "This range of the boolean input registers is reserved for"           \
         "FieldBus/PLC interface usage.")                                      \
    RRII(input_bit_registers32_to_63, 102, int,                                \
         "General purpose bits "                                               \
         "This range of the boolean input registers is reserved for"           \
         "FieldBus/PLC interface usage.")                                      \
    RRII(input_bit_registers64_to_127, 103, int64_t,                           \
         "64 general purpose bits "                                            \
         "X: [64..127] - The upper range of the boolean input registers can"   \
         "be used by external RTDE clients (i.e AUBOCAPS).")                   \
    RRII(input_int_registers_0, 104, int,                                      \
         "48 general purpose integer registers "                               \
         "X: [0..23] - The lower range of the integer input registers is"      \
         "reserved for FieldBus/PLC interface usage. X: [24..47] - The"        \
         "upper range of the integer input registers can be used by"           \
         "external RTDE clients (i.e AUBOCAPS).")                              \
    RRII(input_float_registers_0, 168, float,                                  \
         "48 general purpose integer registers "                               \
         "X: [0..23] - The lower range of the integer input registers is"      \
         "reserved for FieldBus/PLC interface usage. X: [24..47] - The"        \
         "upper range of the integer input registers can be used by"           \
         "external RTDE clients (i.e AUBOCAPS).")                              \
    RRII(input_double_registers_0, 232, double,                                \
         "48 general purpose double registers"                                 \
         "X: [0..23]  - The lower range of the double input registers is "     \
         "reserved for FieldBus/PLC interface usage."                          \
         "X: [24..47] - The upper range of the double input registers can be " \
         "used by external RTDE clients (i.e AUBOCAPS).")                      \
    ROBOT_RTDE_INPUT_MAP(R1, 1000)                                             \
    ROBOT_RTDE_INPUT_MAP(R2, 2000)                                             \
    ROBOT_RTDE_INPUT_MAP(R3, 3000)                                             \
    ROBOT_RTDE_INPUT_MAP(R4, 4000)

#define RTDE_OUTPUT_MAP                                                       \
    RRII(timestamp, 100, double,                                              \
         "Time elapsed since the controller was started [s]")                 \
    RRII(line_number, 101, int, "line number set by setPlanContext")          \
    RRII(runtime_state, 102, RuntimeState, "Program state")                   \
    RRII(output_bit_registers_0_to_63, 103, int64_t,                          \
         "64 [000..063] General purpose bits")                                \
    RRII(output_bit_registers_64_to_127, 104, int64_t,                        \
         "64 [064..127] general purpose bits")                                \
    RRII(output_int_registers_0, 105, int,                                    \
         "48 general purpose integer registers"                               \
         "X: [0..23] - The lower range of the integer output registers is"    \
         "reserved for FieldBus/PLC interface usage. "                        \
         "X: [24..47] - The upper range of the integer output registers can " \
         "be used by external RTDE clients (i.e AUBOCAPS).")                  \
    RRII(output_float_registers_0, 169, int,                                  \
         "48 general purpose integer registers"                               \
         "X: [0..23] - The lower range of the integer output registers is"    \
         "reserved for FieldBus/PLC interface usage. "                        \
         "X: [24..47] - The upper range of the integer output registers can " \
         "be used by external RTDE clients (i.e AUBOCAPS).")                  \
    RRII(output_double_registers_0, 233, double,                              \
         "48 general purpose double registers"                                \
         "X: [0..23] - The lower range of the double output registers is"     \
         "reserved for FieldBus/PLC interface usage. "                        \
         "X: [24..47] - The upper range of the double output registers can "  \
         "be used by external RTDE clients (i.e AUBOCAPS).")                  \
    RRII(input_bit_registers_r0_to_63, 297, int64_t,                          \
         "[0..63] General purpose bits This range of the boolean output"      \
         "registers is reserved for FieldBus/PLC interface usage.")           \
    RRII(input_bit_registers_r64_to_127, 298, int64_t,                        \
         "64 [64..127] general purpose bits")                                 \
    RRII(input_int_registers_r0, 299, int,                                    \
         "([0 .. 48]) 48 general purpose integer registers"                   \
         "X: [0..23] - The lower range of the integer input registers is"     \
         "reserved for FieldBus/PLC interface usage. X: [24..47] - The"       \
         "upper range of the integer input registers can be used by"          \
         "external RTDE clients (i.e AUBOCAPS).")                             \
    RRII(input_float_registers_r0, 363, int,                                  \
         "([0 .. 48]) 48 general purpose integer registers"                   \
         "X: [0..23] - The lower range of the integer input registers is"     \
         "reserved for FieldBus/PLC interface usage. X: [24..47] - The"       \
         "upper range of the integer input registers can be used by"          \
         "external RTDE clients (i.e AUBOCAPS).")                             \
    RRII(input_double_registers_r0, 427, double,                              \
         "([0 .. 48]) 48 general purpose double registers"                    \
         "X: [0..23] - The lower range of the double input registers is"      \
         "reserved for FieldBus/PLC interface usage. X: [24..47] - The"       \
         "upper range of the double input registers can be used by"           \
         "external RTDE clients (i.e AUBOCAPS).")                             \
    RRII(modbus_signals, 500, std::vector<int>,                               \
         "Modbus signals from connected modbus slaves")                       \
    RRII(modbus_signals_errors, 501, std::vector<int>,                        \
         "Modbus signals request status from connected modbus slaves")        \
    ROBOT_RTDE_OUTPUT_MAP(R1, 1000)                                           \
    ROBOT_RTDE_OUTPUT_MAP(R2, 2000)                                           \
    ROBOT_RTDE_OUTPUT_MAP(R3, 3000)                                           \
    ROBOT_RTDE_OUTPUT_MAP(R4, 4000)

#define RRII(i, n, ...) i = n,
enum RtdeInput
{
    RTDE_INPUT_MAP
};
enum RtdeOutput
{
    RTDE_OUTPUT_MAP
};
#undef RRII

#if defined ENABLE_JSON_TYPES
using Json = nlohmann::json;
template <size_t i, size_t N>
struct RtdeInputHandlerForRobot
{
    int operator()(AuboApiPtr /*interface*/, const Json & /*j*/) { return -1; }
};

template <size_t i, size_t N>
struct RtdeOutputHandlerForRobot
{
    int operator()(AuboApiPtr /*interface*/, Json & /*j*/) { return -1; }
};

template <size_t i>
struct RtdeInputHandlerForRobot<i, R1_speed_slider_fraction>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);
        if (robot) {
            auto f = j.get<double>();

            int ret = robot->getMotionControl()->setSpeedFraction(f);
            return ret;
        }
        return -1;
    }
};

template <size_t i>
struct RtdeInputHandlerForRobot<i, R1_standard_digital_output>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto p1 = j[0].get<int>();
            auto p2 = j[2].get<bool>();
            int ret = robot->getIoControl()->setStandardDigitalOutput(p1, p2);
            return ret;
        }
        return -1;
    };
};

template <size_t i>
struct RtdeInputHandlerForRobot<i, R1_configurable_digital_output>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            int ret = robot->getIoControl()->setStandardDigitalOutput(1, false);
            return ret;
        }
        return -1;
    };
};

template <size_t i>
struct RtdeInputHandlerForRobot<i, R1_tool_digital_output>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            int ret = robot->getIoControl()->setToolDigitalOutput(0, true);
        }
        return -1;
    };
};

template <size_t i>
struct RtdeInputHandlerForRobot<i, R1_standard_analog_output_type>
{
    int operator()(AuboApiPtr interface, const Json &j) { return -1; };
};

template <size_t i>
struct RtdeInputHandlerForRobot<i, R1_standard_analog_output>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto robot_names = interface->getRobotNames();
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            robot->getIoControl()->setStandardAnalogOutput(0, 0);
        }
        return -1;
    };
};

template <size_t i>
struct RtdeInputHandlerForRobot<i, R1_debug>
{
    int operator()(AuboApiPtr interface, const Json &j) { return 1; };
};

#define SIMPLE_ROBOT_STATE_OUT_FUNC(n, func)                           \
    template <size_t i>                                                \
    struct RtdeOutputHandlerForRobot<i, n>                             \
    {                                                                  \
        int operator()(AuboApiPtr interface, Json &j)                  \
        {                                                              \
            auto robot_names = interface->getRobotNames();             \
            if (robot_names.size() < (i + 1)) {                        \
                return -1;                                             \
            }                                                          \
            auto robot = interface->getRobotInterface(robot_names[i]); \
                                                                       \
            if (robot) {                                               \
                j = robot->getRobotState()->func();                    \
            }                                                          \
            return 0;                                                  \
        };                                                             \
    };

SIMPLE_ROBOT_STATE_OUT_FUNC(R1_target_q, getJointTargetPositions);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_target_qd, getJointTargetSpeeds);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_target_qdd, getJointTargetAccelerations);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_actual_q, getJointPositions);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_actual_qd, getJointSpeeds);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_actual_qdd, getJointAccelerations);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_actual_current, getJointCurrents);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_actual_TCP_pose, getTcpPose);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_joint_temperatures, getJointTemperatures);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_robot_mode, getRobotModeType);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_safety_mode, getSafetyModeType);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_joint_mode, getJointState);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_actual_main_voltage, getMainVoltage);
SIMPLE_ROBOT_STATE_OUT_FUNC(R1_actual_robot_voltage, getRobotVoltage);
#undef SIMPLE_ROBOT_STATE_OUT_FUNC

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_message>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        uint64_t last_time = 0;

        auto ainfos = robot->getTrace()->peek(1, last_time);
        if (ainfos.size()) {
            j = ainfos;
            return -1;
        }
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_motion_progress>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            j = robot->getMotionControl()->getProgress();
        }
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_standard_digital_input_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            j = robot->getIoControl()->getStandardDigitalInputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump()
        //          << std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_tool_digital_input_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto num = robot->getIoControl()->getToolDigitalInputNum();
            if (num > 64) {
            }

            j = robot->getIoControl()->getToolDigitalInputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump() <<
        // std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_configurable_digital_input_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto num = robot->getIoControl()->getConfigurableDigitalInputNum();
            if (num > 64) {
            }

            j = robot->getIoControl()->getConfigurableDigitalInputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump() <<
        // std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_link_digital_input_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto num = robot->getIoControl()->getStaticLinkInputNum();
            if (num > 32) {
            }

            j = robot->getIoControl()->getStaticLinkInputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump() <<
        // std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_standard_digital_output_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto num = robot->getIoControl()->getStandardDigitalOutputNum();
            if (num > 64) {
            }

            j = robot->getIoControl()->getStandardDigitalOutputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump() <<
        // std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_tool_digital_output_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto num = robot->getIoControl()->getToolDigitalOutputNum();
            if (num > 64) {
            }

            j = robot->getIoControl()->getToolDigitalOutputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump() <<
        // std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_configurable_digital_output_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto num = robot->getIoControl()->getConfigurableDigitalOutputNum();
            if (num > 64) {
            }

            j = robot->getIoControl()->getConfigurableDigitalOutputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump() <<
        // std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_link_digital_output_bits>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);

        if (robot) {
            auto num = robot->getIoControl()->getStaticLinkOutputNum();
            if (num > 32) {
            }

            j = robot->getIoControl()->getStaticLinkOutputs();
        }
        // std::cout << "RTDE R1_standard_digital_input_bits " << j.dump() <<
        // std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_standard_analog_input_values>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);
        if (robot) {
            auto num = robot->getIoControl()->getStandardAnalogInputNum();
            std::vector<double> data;
            data.resize(num);
            for (int x = 0; x < num; x++) {
                data[x] = robot->getIoControl()->getStandardAnalogInput(x);
            }
            j = data;
        }
        // std::cout << "RTDE analog_input_values " << j.dump() << std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_tool_analog_input_values>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);
        if (robot) {
            auto num = robot->getIoControl()->getToolAnalogInputNum();
            std::vector<double> data;
            data.resize(num);
            for (int x = 0; x < num; x++) {
                data[x] = robot->getIoControl()->getToolAnalogInput(x);
            }
            j = data;
        }
        // std::cout << "RTDE analog_input_values " << j.dump() << std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_standard_analog_output_values>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);
        if (robot) {
            auto num = robot->getIoControl()->getStandardAnalogOutputNum();
            std::vector<double> data;
            data.resize(num);
            for (int x = 0; x < num; x++) {
                data[x] = robot->getIoControl()->getStandardAnalogOutput(x);
            }
            j = data;
        }
        // std::cout << "RTDE analog_input_values " << j.dump() << std::endl;
        return 0;
    };
};

template <size_t i>
struct RtdeOutputHandlerForRobot<i, R1_tool_analog_output_values>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        auto robot_names = interface->getRobotNames();
        if (robot_names.size() < (i + 1)) {
            return -1;
        }
        auto robot = interface->getRobotInterface(robot_names[i]);
        if (robot) {
            auto num = robot->getIoControl()->getToolAnalogInputNum();
            std::vector<double> data;
            data.resize(num);
            for (int x = 0; x < num; x++) {
                data[x] = robot->getIoControl()->getToolAnalogInput(x);
            }
            j = data;
        }
        // std::cout << "RTDE analog_input_values " << j.dump() << std::endl;
        return 0;
    };
};

template <size_t N>
struct RtdeInputHandler
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        constexpr size_t robot_index = N / 1000;
        constexpr size_t robot_cmd = N % 1000 + 1000;
        if constexpr (robot_index >= 1 && robot_index <= 4) {
            return RtdeInputHandlerForRobot<robot_index - 1, robot_cmd>()(
                interface, j);
        }
        return -1;
    }
};

template <size_t N>
struct RtdeOutputHandler
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        constexpr size_t robot_index = N / 1000;
        constexpr size_t robot_cmd = N % 1000 + 1000;
        if constexpr (robot_index >= 1 && robot_index <= 4) {
            return RtdeOutputHandlerForRobot<robot_index - 1, robot_cmd>()(
                interface, j);
        }
        return -1;
    }
};

template <>
struct RtdeInputHandler<input_bit_registers0_to_31>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto value = j.get<int>();
        for (int i = 0; i < 32; i++) {
            interface->getRegisterControl()->setBoolInput(
                i, (value & (uint32_t)1 << i));
        }
        return 1;
    };
};

template <>
struct RtdeInputHandler<input_bit_registers32_to_63>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto value = j.get<int>();
        for (int i = 32; i < 64; i++) {
            interface->getRegisterControl()->setBoolInput(
                i, (value & (uint32_t)1 << (i - 32)));
        }
        return 1;
    };
};

template <>
struct RtdeInputHandler<input_bit_registers64_to_127>
{
    int operator()(AuboApiPtr interface, const Json &j)
    {
        auto value = j.get<int64_t>();
        for (int i = 64; i < 128; i++) {
            interface->getRegisterControl()->setBoolInput(
                i, (value & (uint64_t)1 << (i - 64)));
        }
        return 1;
    };
};

template <>
struct RtdeOutputHandler<runtime_state>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        j = interface->getRuntimeMachine()->getStatus();
        return 0;
    }
};

template <>
struct RtdeOutputHandler<line_number>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        int tid, lineno;
        std::string comment;
        std::tie(tid, lineno, comment) =
            interface->getRuntimeMachine()->getPlanContext();
        j = lineno;
        return 0;
    }
};

template <>
struct RtdeOutputHandler<output_bit_registers_0_to_63>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        uint64_t value = 0;

        for (int i = 0; i < 64; i++) {
            if (interface->getRegisterControl()->getBoolOutput(i)) {
                value |= (uint64_t)1 << i;
            }
        }

        j = value;
        return 0;
    }
};

template <>
struct RtdeOutputHandler<output_bit_registers_64_to_127>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        uint64_t value = 0;
        for (int i = 64; i < 128; i++) {
            if (interface->getRegisterControl()->getBoolOutput(i)) {
                value |= (uint64_t)1 << (i - 64);
            }
        }
        j = value;
        return 0;
    }
};

template <>
struct RtdeOutputHandler<input_bit_registers_r0_to_63>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        uint64_t value = 0;

        for (int i = 0; i < 64; i++) {
            if (interface->getRegisterControl()->getBoolInput(i)) {
                value |= (uint64_t)1 << i;
            }
        }

        j = value;
        return 0;
    }
};

template <>
struct RtdeOutputHandler<input_bit_registers_r64_to_127>
{
    int operator()(AuboApiPtr interface, Json &j)
    {
        uint64_t value = 0;
        for (int i = 64; i < 128; i++) {
            if (interface->getRegisterControl()->getBoolInput(i)) {
                value |= (uint64_t)1 << (i - 64);
            }
        }
        j = value;
        return 0;
    }
};

/// Server RTDE Input
inline int parseToServer(int n, AuboApiPtr interface, const Json &j)
{
    interface->getRuntimeMachine()->setInstructionAllowed(true);
    int retval = 0;
    if (n >= RtdeInput::input_int_registers_0 &&
        n < RtdeInput::input_int_registers_0 + 64) {
        interface->getRegisterControl()->setInt32Input(
            n - RtdeInput::input_int_registers_0, j.get<int>());
        return 0;
    } else if (n >= RtdeInput::input_float_registers_0 &&
               n < RtdeInput::input_float_registers_0 + 64) {
        interface->getRegisterControl()->setFloatInput(
            n - RtdeInput::input_float_registers_0, j.get<float>());
        return 0;
    } else if (n >= RtdeInput::input_double_registers_0 &&
               n < RtdeInput::input_double_registers_0 + 64) {
        interface->getRegisterControl()->setDoubleInput(
            n - RtdeInput::input_double_registers_0, j.get<double>());
        return 0;
    }
    interface->getRuntimeMachine()->setInstructionAllowed(false);

    switch (n) {
#define RRII(I, N, ...)                             \
    case I:                                         \
        return RtdeInputHandler<I>()(interface, j); \
        break;
        RTDE_INPUT_MAP
#undef RRII
    default:
        retval = -1;
    }

    return retval;
}

/// Server RTDE Output
inline int parseFromServer(int n, AuboApiPtr interface, Json &j)
{
    int retval = 0;
    if (n >= RtdeOutput::output_int_registers_0 &&
        n < RtdeOutput::output_int_registers_0 + 64) {
        j = interface->getRegisterControl()->getInt32Output(
            n - RtdeOutput::output_int_registers_0);
        return 0;
    } else if (n >= RtdeOutput::output_float_registers_0 &&
               n < RtdeOutput::output_float_registers_0 + 64) {
        j = interface->getRegisterControl()->getFloatOutput(
            n - RtdeOutput::output_float_registers_0);
        return 0;
    } else if (n >= RtdeOutput::output_double_registers_0 &&
               n < RtdeOutput::output_double_registers_0 + 64) {
        j = interface->getRegisterControl()->getDoubleOutput(
            n - RtdeOutput::output_double_registers_0);
        return 0;
    } else if (n >= RtdeOutput::input_int_registers_r0 &&
               n < RtdeOutput::input_int_registers_r0 + 64) {
        j = interface->getRegisterControl()->getInt32Input(
            n - RtdeOutput::input_int_registers_r0);
        return 0;
    } else if (n >= RtdeOutput::input_float_registers_r0 &&
               n < RtdeOutput::input_float_registers_r0 + 64) {
        j = interface->getRegisterControl()->getFloatInput(
            n - RtdeOutput::input_float_registers_r0);
        return 0;
    } else if (n >= RtdeOutput::input_double_registers_r0 &&
               n < RtdeOutput::input_double_registers_r0 + 64) {
        j = interface->getRegisterControl()->getDoubleInput(
            n - RtdeOutput::input_double_registers_r0);
        return 0;
    } else if (n == RtdeOutput::modbus_signals) {
        j = interface->getRegisterControl()->modbusGetSignalValues();
        return 0;
    } else if (n == RtdeOutput::modbus_signals_errors) {
        j = interface->getRegisterControl()->modbusGetSignalErrors();
        return 0;
    }

    switch (n) {
#define RRII(I, N, ...)                              \
    case I:                                          \
        return RtdeOutputHandler<I>()(interface, j); \
        break;
        RTDE_OUTPUT_MAP
#undef RRII
    default:
        retval = -1;
    }

    return retval;
}
#endif
} // namespace common_interface
} // namespace arcs

#endif
