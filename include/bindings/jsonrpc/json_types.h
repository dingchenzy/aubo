#ifndef COMMON_INTERFACE_TYPE_DEF_JSON_H
#define COMMON_INTERFACE_TYPE_DEF_JSON_H
#ifdef max
#undef max
#endif
#include <random>
#include <sstream>
#include <aubo/type_def.h>
#include <nlohmann/json.hpp>

namespace arcs {
namespace common_interface {

// RuntimeState
NLOHMANN_JSON_SERIALIZE_ENUM(RuntimeState,
                             { { RuntimeState::Running, "Running" },
                               { RuntimeState::Retracting, "Retracting" },
                               { RuntimeState::Pausing, "Pausing" },
                               { RuntimeState::Paused, "Paused" },
                               { RuntimeState::Stopping, "Stopping" },
                               { RuntimeState::Stopped, "Stopped" },
                               { RuntimeState::Aborting, "Aborting" } })

// RobotModeType
NLOHMANN_JSON_SERIALIZE_ENUM(
    RobotModeType, { { RobotModeType::NoController, "NoController" },
                     { RobotModeType::Disconnected, "Disconnected" },
                     { RobotModeType::ConfirmSafety, "ConfirmSafety" },
                     { RobotModeType::Booting, "Booting" },
                     { RobotModeType::PowerOff, "PowerOff" },
                     { RobotModeType::PowerOn, "PowerOn" },
                     { RobotModeType::Idle, "Idle" },
                     { RobotModeType::BrakeReleasing, "BrakeReleasing" },
                     { RobotModeType::BackDrive, "BackDrive" },
                     { RobotModeType::Running, "Running" },
                     { RobotModeType::Maintaince, "Maintaince" },
                     { RobotModeType::Error, "Error" } })

// SafetyModeType
NLOHMANN_JSON_SERIALIZE_ENUM(
    SafetyModeType,
    { { SafetyModeType::Undefined, "Undefined" },
      { SafetyModeType::Normal, "Normal" },
      { SafetyModeType::ReducedMode, "ReducedMode" },
      { SafetyModeType::Recovery, "Recovery" },
      { SafetyModeType::Violation, "Violation" },
      { SafetyModeType::ProtectiveStop, "ProtectiveStop" },
      { SafetyModeType::SafeguardStop, "SafeguardStop" },
      { SafetyModeType::SystemEmergencyStop, "SystemEmergencyStop" },
      { SafetyModeType::RobotEmergencyStop, "RobotEmergencyStop" },
      { SafetyModeType::Fault, "Fault" } })

// OperationalModeType
NLOHMANN_JSON_SERIALIZE_ENUM(OperationalModeType,
                             { { OperationalModeType::Disabled, "Disabled" },
                               { OperationalModeType::Automatic, "Automatic" },
                               { OperationalModeType::Manual, "Manual" } })

// RobotControlModeType
NLOHMANN_JSON_SERIALIZE_ENUM(RobotControlModeType,
                             { { RobotControlModeType::None, "None" },
                               { RobotControlModeType::Position, "Position" },
                               { RobotControlModeType::Speed, "Speed" },
                               { RobotControlModeType::Servo, "Servo" },
                               { RobotControlModeType::Freedrive, "Freedrive" },
                               { RobotControlModeType::Force, "Force" },
                               { RobotControlModeType::Torque, "Torque" },
                               { RobotControlModeType::Collision,
                                 "Collision" } })

// JointServoModeType
NLOHMANN_JSON_SERIALIZE_ENUM(JointServoModeType,
                             { { JointServoModeType::None, "None" },
                               { JointServoModeType::Open, "Open" },
                               { JointServoModeType::Current, "Current" },
                               { JointServoModeType::Velocity, "Velocity" },
                               { JointServoModeType::Position, "Position" },
                               { JointServoModeType::Pvt, "Pvt" },
                               { JointServoModeType::Torque, "Torque" } })

// JointStateType
NLOHMANN_JSON_SERIALIZE_ENUM(JointStateType,
                             { { JointStateType::Poweroff, "Poweroff" },
                               { JointStateType::Idle, "Idle" },
                               { JointStateType::Fault, "Fault" },
                               { JointStateType::Running, "Running" },
                               { JointStateType::Bootload, "Bootload" } })

// StandardInputAction
NLOHMANN_JSON_SERIALIZE_ENUM(
    StandardInputAction,
    { { StandardInputAction::Default, "Default" },
      { StandardInputAction::Handguide, "Handguide" },
      { StandardInputAction::PowerOn, "PowerOn" },
      { StandardInputAction::StartProgram, "StartProgram" },
      { StandardInputAction::StopProgram, "StopProgram" },
      { StandardInputAction::PauseProgram, "PauseProgram" },
      { StandardInputAction::ResumeProgram, "ResumeProgram" },
      { StandardInputAction::SlowDown1, "SlowDown1" },
      { StandardInputAction::SlowDown2, "SlowDown2" },
      { StandardInputAction::SafeStop, "SafeStop" },
      { StandardInputAction::RunningGuard, "RunningGuard" },
      { StandardInputAction::PowerOff, "PowerOff" },
      { StandardInputAction::MoveToFirstPoint, "MoveToFirstPoint" },
      { StandardInputAction::PopupDismiss, "PopupDismiss" },
      { StandardInputAction::GoHome, "GoHome" } })

// StandardOutputRunState
NLOHMANN_JSON_SERIALIZE_ENUM(
    StandardOutputRunState,
    { { StandardOutputRunState::None, "None" },
      { StandardOutputRunState::Handguiding, "Handguiding" },
      { StandardOutputRunState::PowerOn, "PowerOn" },
      { StandardOutputRunState::StopLow, "StopLow" },
      { StandardOutputRunState::StopHigh, "StopHigh" },
      { StandardOutputRunState::RunningHigh, "RunningHigh" },
      { StandardOutputRunState::PausedHigh, "PausedHigh" },
      { StandardOutputRunState::AtHome, "AtHome" } })

// SafetyInputAction
NLOHMANN_JSON_SERIALIZE_ENUM(
    SafetyInputAction,
    { { SafetyInputAction::Unassigned, "Unassigned" },
      { SafetyInputAction::EmergencyStop, "EmergencyStop" },
      { SafetyInputAction::ReducedMode, "ReducedMode" },
      { SafetyInputAction::SafeguardStop, "SafeguardStop" },
      { SafetyInputAction::SafeguardReset, "SafeguardReset" },
      { SafetyInputAction::ThreePositionSwitch, "ThreePositionSwitch" },
      { SafetyInputAction::HandGuide, "HandGuide" },
      { SafetyInputAction::AutomaticModeSafeguardStop,
        "AutomaticModeSafeguardStop" },
      { SafetyInputAction::AutomaticModeSafeguardReset,
        "AutomaticModeSafeguardReset" },
      { SafetyInputAction::OperationalMode, "OperationalMode" } })

// SafetyOutputRunState
NLOHMANN_JSON_SERIALIZE_ENUM(
    SafetyOutputRunState,
    { { SafetyOutputRunState::Unassigned, "Unassigned" },
      { SafetyOutputRunState::SystemEmergencyStop, "SystemEmergencyStop" },
      { SafetyOutputRunState::RobotMoving, "RobotMoving" },
      { SafetyOutputRunState::RobotNotMoving, "RobotNotMoving" },
      { SafetyOutputRunState::ReducedMode, "ReducedMode" },
      { SafetyOutputRunState::NotReducedMode, "NotReducedMode" },
      { SafetyOutputRunState::SafeHome, "SafeHome" } })

// TaskFrameType
NLOHMANN_JSON_SERIALIZE_ENUM(TaskFrameType,
                             { { TaskFrameType::NONE, "NONE" },
                               { TaskFrameType::FRAME_FORCE, "FRAME_FORCE" },
                               { TaskFrameType::POINT_FORCE, "POINT_FORCE" },
                               { TaskFrameType::MOTION_FORCE,
                                 "MOTION_FORCE" } })

// OperationalModeType
NLOHMANN_JSON_SERIALIZE_ENUM(TraceLevel, { { TraceLevel::FATAL, "FATAL" },
                                           { TraceLevel::ERROR, "ERROR" },
                                           { TraceLevel::WARNING, "WARNING" },
                                           { TraceLevel::INFO, "INFO" },
                                           { TraceLevel::DEBUG, "DEBUG" } })

inline void to_json(nlohmann::json &j, const RobotMsg &p)
{
    j = nlohmann::json{ { "timestamp", p.timestamp },
                        { "level", p.level },
                        { "code", p.code },
                        { "source", p.source },
                        { "args", p.args } };
}

inline void from_json(const nlohmann::json &j, RobotMsg &p)
{
    j.at("level").get_to(p.level);
    j.at("timestamp").get_to(p.timestamp);
    j.at("code").get_to(p.code);
    j.at("source").get_to(p.source);
    j.at("args").get_to(p.args);
}

inline void to_json(nlohmann::json &j, const RtdeRecipe &p)
{
    j = nlohmann::json{ { "to_server", p.to_server },
                        { "chanel", p.chanel },
                        { "frequency", p.frequency },
                        { "trigger", p.trigger },
                        { "segments", p.segments } };
}

inline void from_json(const nlohmann::json &j, RtdeRecipe &p)
{
    j.at("to_server").get_to(p.to_server);
    j.at("chanel").get_to(p.chanel);
    j.at("frequency").get_to(p.frequency);
    j.at("trigger").get_to(p.trigger);
    j.at("segments").get_to(p.segments);
}

inline void to_json(nlohmann::json &j, const RobotSafetyParameterRange &p)
{
    nlohmann::json params;
    nlohmann::json trigger_planes;
    nlohmann::json tools;

    for (int i = 0; i < SAFETY_PARAM_SELECT_NUM; i++) {
        nlohmann::json pn;

        pn["power"] = p.params[i].power;
        pn["momentum"] = p.params[i].momentum;
        pn["stop_time"] = p.params[i].stop_time;
        pn["stop_distance"] = p.params[i].stop_distance;
        pn["tcp_speed"] = p.params[i].tcp_speed;
        pn["elbow_speed"] = p.params[i].elbow_speed;
        pn["tcp_force"] = p.params[i].tcp_force;
        pn["elbow_force"] = p.params[i].elbow_force;
        pn["qmin"] = p.params[i].qmin;
        pn["qmax"] = p.params[i].qmax;
        pn["qdmax"] = p.params[i].qdmax;
        pn["joint_torque"] = p.params[i].joint_torque;
        pn["tool_orientation"] = p.params[i].tool_orientation;
        pn["tool_deviation"] = p.params[i].tool_deviation;
        for (int n = 0; n < SAFETY_PLANES_NUM; n++) {
            pn["planes"].push_back(p.params[i].planes[n]);
        }

        params.push_back(pn);
    }

    for (int i = 0; i < SAFETY_PLANES_NUM; i++) {
        nlohmann::json pn;
        pn["plane"] = p.trigger_planes[i].plane;
        pn["restrict_elbow"] = p.trigger_planes[i].restrict_elbow;
        pn["param_select"] = p.trigger_planes[i].param_select;

        trigger_planes.push_back(pn);
    }

    for (int i = 0; i < TOOL_CONFIGURATION_NUM; i++) {
        tools.push_back(p.tools[i]);
    }

    j["crc32"] = p.crc32;
    j["params"] = params;
    j["trigger_planes"] = trigger_planes;
    j["tools"] = tools;
    j["tool_inclination"] = p.tool_inclination;
    j["tool_azimuth"] = p.tool_azimuth;
    j["safety_home"] = p.safety_home;

    j["safety_input_emergency_stop"] = p.safety_input_emergency_stop;
    j["safety_input_safegurd_stop"] = p.safety_input_safegurd_stop;
    j["safety_input_safeguard_reset"] = p.safety_input_safeguard_reset;
    j["safety_input_auto_safegurd_stop"] = p.safety_input_auto_safegurd_stop;
    j["safety_input_auto_safeguard_reset"] =
        p.safety_input_auto_safeguard_reset;
    j["safety_input_three_position_switch"] =
        p.safety_input_three_position_switch;
    j["safety_input_operational_mode"] = p.safety_input_operational_mode;
    j["safety_input_reduced_mode"] = p.safety_input_reduced_mode;
    j["safety_input_handguide"] = p.safety_input_handguide;
    j["safety_output_safe_home"] = p.safety_output_safe_home;
    j["safety_output_reduced_mode"] = p.safety_output_reduced_mode;
    j["safety_output_not_reduced_mode"] = p.safety_output_not_reduced_mode;
    j["safety_output_emergency_stop"] = p.safety_output_emergency_stop;
    j["safety_output_robot_moving"] = p.safety_output_robot_moving;
    j["safety_output_robot_steady"] = p.safety_output_robot_steady;
}

inline void from_json(const nlohmann::json &j, RobotSafetyParameterRange &p)
{
    j.at("crc32").get_to(p.crc32);

    auto params = j.at("params");
    for (int i = 0; i < SAFETY_PARAM_SELECT_NUM; i++) {
        params[i].at("power").get_to(p.params[i].power);
        params[i].at("momentum").get_to(p.params[i].momentum);
        params[i].at("stop_time").get_to(p.params[i].stop_time);
        params[i].at("stop_distance").get_to(p.params[i].stop_distance);
        params[i].at("tcp_speed").get_to(p.params[i].tcp_speed);
        params[i].at("elbow_speed").get_to(p.params[i].elbow_speed);
        params[i].at("tcp_force").get_to(p.params[i].tcp_force);
        params[i].at("elbow_force").get_to(p.params[i].elbow_force);
        params[i].at("qmin").get_to(p.params[i].qmin);
        params[i].at("qmax").get_to(p.params[i].qmax);
        params[i].at("qdmax").get_to(p.params[i].qdmax);
        params[i].at("joint_torque").get_to(p.params[i].joint_torque);
        params[i].at("tool_orientation").get_to(p.params[i].tool_orientation);
        params[i].at("tool_deviation").get_to(p.params[i].tool_deviation);

        auto planes = params[i].at("planes");

        for (int n = 0; n < SAFETY_PLANES_NUM; n++) {
            planes[n].get_to(p.params[i].planes[n]);
        }
    }

    auto trigger_planes = j.at("trigger_planes");
    for (int i = 0; i < SAFETY_PLANES_NUM; i++) {
        trigger_planes[i].at("plane").get_to(p.trigger_planes[i].plane);
        trigger_planes[i]
            .at("restrict_elbow")
            .get_to(p.trigger_planes[i].restrict_elbow);
        trigger_planes[i]
            .at("param_select")
            .get_to(p.trigger_planes[i].param_select);
    }

    auto tools = j.at("tools");
    for (int i = 0; i < TOOL_CONFIGURATION_NUM; i++) {
        tools[i].get_to(p.tools[i]);
    }

    j.at("crc32").get_to(p.crc32);
    j.at("tool_inclination").get_to(p.tool_inclination);
    j.at("tool_azimuth").get_to(p.tool_azimuth);
    j.at("safety_home").get_to(p.safety_home);

    j.at("safety_input_emergency_stop").get_to(p.safety_input_emergency_stop);
    j.at("safety_input_safegurd_stop").get_to(p.safety_input_safegurd_stop);
    j.at("safety_input_safeguard_reset").get_to(p.safety_input_safeguard_reset);
    j.at("safety_input_auto_safegurd_stop")
        .get_to(p.safety_input_auto_safegurd_stop);
    j.at("safety_input_auto_safeguard_reset")
        .get_to(p.safety_input_auto_safeguard_reset);
    j.at("safety_input_three_position_switch")
        .get_to(p.safety_input_three_position_switch);
    j.at("safety_input_operational_mode")
        .get_to(p.safety_input_operational_mode);
    j.at("safety_input_reduced_mode").get_to(p.safety_input_reduced_mode);
    j.at("safety_input_handguide").get_to(p.safety_input_handguide);
    j.at("safety_output_safe_home").get_to(p.safety_output_safe_home);
    j.at("safety_output_reduced_mode").get_to(p.safety_output_reduced_mode);
    j.at("safety_output_not_reduced_mode")
        .get_to(p.safety_output_not_reduced_mode);
    j.at("safety_output_emergency_stop").get_to(p.safety_output_emergency_stop);
    j.at("safety_output_robot_moving").get_to(p.safety_output_robot_moving);
    j.at("safety_output_robot_steady").get_to(p.safety_output_robot_steady);
}

inline void to_json(nlohmann::json &j, const CircleParameters &p)
{
    j = nlohmann::json{ { "pose_via", p.pose_via },
                        { "pose_to", p.pose_to },
                        { "a", p.a },
                        { "v", p.v },
                        { "blend_radius", p.blend_radius },
                        { "duration", p.duration },
                        { "helix", p.helix },
                        { "spiral", p.spiral },
                        { "direction", p.direction },
                        { "loop_times", p.loop_times } };
}

inline void from_json(const nlohmann::json &j, CircleParameters &p)
{
    j.at("pose_via").get_to(p.pose_via);
    j.at("pose_to").get_to(p.pose_to);
    j.at("a").get_to(p.a);
    j.at("v").get_to(p.v);
    j.at("blend_radius").get_to(p.blend_radius);
    j.at("duration").get_to(p.duration);
    j.at("helix").get_to(p.helix);
    j.at("spiral").get_to(p.spiral);
    j.at("direction").get_to(p.direction);
    j.at("loop_times").get_to(p.loop_times);
}

} // namespace common_interface
} // namespace arcs
#endif
