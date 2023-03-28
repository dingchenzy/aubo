#ifndef AUBO_SDK_RTM_ERROR_H
#define AUBO_SDK_RTM_ERROR_H

// clang-format off

#define RTM_ERRORS \
    _D(ROBOT_BE_PULLING, 30001, "Something is pulling the robot.","Please check TCP configuration,payload and mounting settings") \
    _D(PSTOP_ELBOW_POS, 30002, "Protective Stop: Elbow position close to safety plane limits.","Please move robot Elbow joint away from the safety plane") \
    _D(PSTOP_STOP_TIME, 30003, "Protective Stop: Exceeding user safety settings for stopping time.","(a) Check speeds and accelerations in the program (b) Check usage of TCP,payload and CoG correctly (c) Check external equipmentactivation if correctly set") \
    _D(PSTOP_STOP_DISTANCE, 30004, "Protective Stop: Exceeding user safety settings for stopping distance.","(a) Check speeds and accelerations in the program (b) Check usage of TCP,payload and CoG correctly (c) Check external equipmentactivation if correctly set") \
    _D(PSTOP_CLAMP, 30005, "Protective Stop: Danger of clamping between the Robot’s lower arm and tool.","(a) Check speeds and accelerations in the program (b) Check usage of TCP,payload and CoG correctly (c) Check external equipmentactivation if correctly set") \
    _D(PSTOP_POS_LIMIT, 30006, "Protective Stop: Position close to joint limits", "suggest...") \
    _D(PSTOP_ORI_LIMIT, 30007, "Protective Stop: Tool orientation close to limits", "suggest...") \
    _D(PSTOP_PLANE_LIMIT, 30008, "Protective Stop: Position close to safety plane limits", "suggest...") \
    _D(PSTOP_POS_DEVIATE, 30009, "Protective Stop: Position deviates from path", "Check payload, center of gravity and acceleration settings.") \
    _D(JOINT_CHK_PAYLOAD, 30010, "Joint " _PH1_ ": Check payload, center of gravity and acceleration settings. Log screen may contain additional information.", "suggest...") \
    _D(PSTOP_SINGULARITY, 30011, "Protective Stop: Position in singularity.","Please use MoveJ or change the motion") \
    _D(PSTOP_CANNOT_MAINTAIN, 30012, "Protective Stop: Robot cannot maintain its position, check if payload is correct", "suggest...") \
    _D(PSTOP_WRONG_PAYLOAD, 30013, "Protective Stop: Wrong payload or mounting detected, or something is pushing the robot when entering Freedrive mode","Verify that the TCP configuration and mounting in the used installation is correct") \
    _D(PSTOP_JOINT_COLLISION, 30014, "Protective Stop: Collision detected by joint", "Make sure no objects are in the path of the robot and resume the program") \
    _D(PSTOP_POS_DISAGREE, 30015, "Protective stop: The robot was powered off last time due to a joint position disagreement."," (a) Verify that the robot position in the 3D graphics matches the real robot, to ensure that the encoders function before releasing the brakes. Stand back and monitor the robot performing its first program cycle as expected. (b) If the position is not correct, the robot must be repaired. In this case, click Power Off Robot. (c) If the position is correct, please tick the check box below the 3D graphics and click Robot Position Verified") \
    _D(PSTOP_LARGE_MOVE, 30016, "Protective stop: Large movement of the robot detected while it was powered off. The joints were moved while it was powered off, or the encoders do not function", "suggest...") \
    _D(TARGET_POS_SUDDEN_CHG, 30017, "Sudden change in target position", "suggest...") \
    _D(SUDDEN_STOP, 30018, "Sudden stop."," To abort a motion, use \"stopj\" or \"stopl\" script commands to generate a smooth deceleration before using \"wait\". Avoid aborting motions between waypoints with blend”") \
    _D(ROBOT_STOP_ABNORMAL, 30019, "Robot has not stopped in the allowed reaction and braking time", "suggest...") \
    _D(PROG_INVALID_SETP, 30020, "Robot program resulted in invalid setpoint.", "Please review waypoints in the program") \
    _D(BLEND_INVALID_SETP, 30021, "Blending failed and resulted in an invalid setpoint.", "Try changing the blend radius or contact technical support") \
    _D(APPROACH_SINGULARITY, 30022, "Robot approaching singularity – Acceleration threshold failed.","Review waypoints in the program, try using MoveJ instead of MoveL in the position close to singularity") \
    _D(TSPEED_UNMATCH_POS, 30023, "Target speed does not match target position", "suggest...") \
    _D(INCONSIS_TPOS_SPD, 30024, "Inconsistency between target position and speed", "suggest...") \
    _D(JOINT_TSPD_UNMATCH_POS, 30025, "Target joint speed does not match target joint position change – Joint " _PH1_ "", "suggest...") \
    _D(FIELDBUS_INPUT_DISCONN, 30026, "Fieldbus input disconnected.","Please check fieldbus connections (RTDE, ModBus, EtherNet/IP and Profinet) or disable the fieldbus in the installation. Check RTDE watchdog feature. Check if a URCap is using this feature.") \
    _D(OPMODE_CHANGED, 30027, "Operational mode changed: " _PH1_ "", "suggest...") \
    _D(NO_KIN_CALIB, 30028, "No Kinematic Calibration found (calibration.conf file is either corrupt or missing).","A new kinematics calibration may be needed if the robot needs to improve its kinematics, otherwise, ignore this message)") \
    _D(KIN_CALIB_UNMATCH_JOINT, 30029, "Kinematic Calibration for the robot does not match the joint(s).", "If moving a program from a different robot to this one, rekinematic calibrate the second robot to improve kinematics, otherwise ignore this message.") \
    _D(KIN_CALIB_UNMATCH_ROBOT, 30030, "Kinematic Calibration does not match the robot.","Please check if the serial number of the robot arm matches the Control Box") \
    _D(JOINT_OFFSET_CHANGED, 30031, "Large movement of the robot detected while it was powered off. The joints were moved while it was powered off, or the encoders do not function", "suggest...") \
    _D(OFFSET_CHANGE_HIGH, 30032, "Change in offset is too high", "suggest...") \
    _D(JOINT_SPEED_LIMIT, 30033, "Close to joint speed safety limit.", "Review program speed and acceleration") \
    _D(TOOL_SPEED_LIMIT, 30034, "Close to tool speed safety limit.", "Review program speed and acceleration") \
    _D(MOMENTUM_LIMIT, 30035, "Close to momentum safety limit.", "Review program speed and acceleration") \
    _D(ROBOT_MV_STOP, 30036, "Robot is moving when in Stop Mode", "suggest...") \
    _D(HAND_PROTECTION, 30037, "Hand protection: Tool is too close to the lower arm: " _PH1_ " meter.","(a) Check wrist position. (b) Verify mounting (c) Do a Complete rebooting sequence (d) Update software (e) Contact your local AUBO Robots service provider for assistance") \
    _D(WRONG_SAFETYMODE, 30038, "Wrong safety mode: " _PH1_, "suggest...") \
    _D(SAFETYMODE_CHANGED, 30039, "Safety mode changed: " _PH1_, "suggest...") \
    _D(JOINT_ACC_LIMIT, 30040, "Close to joint acceleration safety limit", "suggest...") \
    _D(TOOL_ACC_LIMIT, 30041, "Close to tool acceleration safety limit", "suggest...") \
    _D(JOINT_TEMPERATURE_LIMIT, 30042, "Joint " _PH1_ " temperature too high(>" _PH2_ "℃)", "suggest...") \
    _D(CONTROL_BOX_TEMPERATURE_LIMIT, 30043, "Control box temperature too high(>" _PH1_ "℃)", "suggest...") \
    _D(ROBOT_EMERGENCY_STOP, 30044, "Robot emergency stop", "suggest...") \
    _D(ROBOTMODE_CHANGED, 30045, "Robot mode changed: " _PH1_, "suggest...") \
    _D(ROBOTMODE_ERROR, 30046, "Wrong robot mode: " _PH1_, "suggest...") \
    _D(POSE_OUT_OF_REACH, 30047, "Target pose [" _PH1_ "] out of reach", "suggest...") \
    _D(TP_PLAN_FAILED, 30048, "Trajectory plan FAILED." , "suggest...") \
    _D(START_FORCE_FAILED, 30049, "Start force control failed, because force sensor does not exist." , "suggest...") \
    _D(OVER_SAFE_PLANE_LIMIT,30050, _PH1_ " axis exceeds the safety plane limit (Move_type:" _PH2_ " id:" _PH3_ ").","Please move the robot to the safety plane range.") \
    _D(POWERON_FAIL_VIOLATION,30051, "Failed to power on because the robot safety mode is in violation", "suggest...") \
    _D(POWERON_FAIL_SYSTEMEMERGENCYSTOP, 30052, "Failed to power on because the robot safety mode is in system emergency stop", "suggest...") \
    _D(POWERON_FAIL_ROBOTEMERGENCYSTOP, 30053, "Failed to power on because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(POWERON_FAIL_FAULT, 30054, "Failed to power on because the robot safety mode is in fault", "suggest...") \
    _D(STARTUP_FAIL_VIOLATION, 30055, "Failed to startup because the robot safety mode is in violation", "suggest...") \
    _D(STARTUP_FAIL_SYSTEMEMERGENCYSTOP, 30056, "Failed to startup because the robot safety mode is in system emergency stop", "suggest...") \
    _D(STARTUP_FAIL_ROBOTEMERGENCYSTOP, 30057, "Failed to startup because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(STARTUP_FAIL_FAULT, 30058, "Failed to startup because the robot safety mode is in fault", "suggest...") \
    _D(BACKDRIVE_FAIL_VIOLATION, 30059, "Failed to backdrive because the robot safety mode is in violation", "suggest...") \
    _D(BACKDRIVE_FAIL_SYSTEMEMERGENCYSTOP, 30060, "Failed to backdrive because the robot safety mode is in system emergency stop", "suggest...") \
    _D(BACKDRIVE_FAIL_ROBOTEMERGENCYSTOP, 30061, "Failed to backdrive because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(BACKDRIVE_FAIL_FAULT, 30062, "Failed to backdrive because the robot safety mode is in fault", "suggest...") \
    _D(SETSIM_FAIL_VIOLATION, 30063, "Switch sim mode failed because the robot safety mode is in violation", "suggest...") \
    _D(SETSIM_FAIL_SYSTEMEMERGENCYSTOP, 30064, "Switch sim mode failed because the robot safety mode is in system emergency stop", "suggest...") \
    _D(SETSIM_FAIL_ROBOTEMERGENCYSTOP, 30065, "Switch sim mode failed because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(SETSIM_FAIL_FAULT, 30066, "Switch sim mode failed because the robot safety mode is in fault", "suggest...") \
    _D(FREEDRIVE_FAIL_VIOLATION, 30067, "Enable handguide mode failed because the robot safety mode is in violation", "suggest...") \
    _D(FREEDRIVE_FAIL_SYSTEMEMERGENCYSTOP, 30068, "Enable handguide mode failed because the robot safety mode is in system emergency stop", "suggest...") \
    _D(FREEDRIVE_FAIL_ROBOTEMERGENCYSTOP, 30069, "Enable handguide mode failed because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(FREEDRIVE_FAIL_FAULT, 30070, "Enable handguide mode failed because the robot safety mode is in fault", "suggest...") \
    _D(UPFIRMWARE_FAIL_VIOLATION, 30071, "Firmware update failed because the robot safety mode is in violation", "suggest...") \
    _D(UPFIRMWARE_FAIL_SYSTEMEMERGENCYSTOP, 30072, "Firmware update failed because the robot safety mode is in system emergency stop", "suggest...") \
    _D(UPFIRMWARE_FAIL_ROBOTEMERGENCYSTOP, 30073, "Firmware update failed because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(UPFIRMWARE_FAIL_FAULT, 30074, "Firmware update failed because the robot safety mode is in fault", "suggest...") \
    _D(SETPERSOSTENT_FAIL_VIOLATION, 30075, "Set persistent parameter failed because the robot safety mode is in violation", "suggest...") \
    _D(SETPERSOSTENT_FAIL_SYSTEMEMERGENCYSTOP, 30076, "Set persistent parameter failed because the robot safety mode is in system emergency stop", "suggest...") \
    _D(SETPERSOSTENT_FAIL_ROBOTEMERGENCYSTOP, 30077, "Set persistent parameter failed because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(SETPERSOSTENT_FAIL_FAULT, 30078, "Set persistent parameter failed because the robot safety mode is in fault", "suggest...") \
    _D(SETPERSOSTENT_FAIL_PARAM_ERR, 30079, "Set persistent parameter failed", "(a) Check the parameter format, whether all are floating point numbers") \
    _D(ROBOT_CABLE_DISCONN, 30080, "Robot cable not connected", "(a) Make sure the cable between Control Box and Robot Arm is correctly connected and it has no damage. (b) Check for loose connections (c) Do a Complete rebooting sequence (d) Update software (e) Contact your local AUBO Robots service provider for assistance Contact your local AUBO Robots service provider for assistance.") \
    _D(TP_TOO_SHORT, 30081, "The generated trajectory is ignored because it is too short", "(a) Please check if the added waypoints are coincident (b) If it is an arc movement, please check whether the three points are collinear") \
    _D(INV_KIN_FAIL, 30082, "Inverse kinematics solution failed. The target pose may be in a singular position or exceed the joint limits", "(a) Change the target pose and try moving again") \
    _D(FREEDRIVE_ENABLED, 30083, "Freedrive status changed to " _PH1_ "", "suggest...") \
    _D(TP_INV_FAIL_REFERENCE_JOINT_OUT_OF_LIMIT, 30084, "Inverse kinematics solution failed. Reference angle [" _PH1_ "] exceeds joint limit [" _PH2_ "].", "suggest...") \
    _D(TP_INV_FAIL_NO_SOLUTION, 30085, "Inverse kinematics solution failed. The reference angle [" _PH1_ "] and the target angle [" _PH2_ "] are used as parameters. there is no solution in the calculation of the inverse solution process.", "suggest...")\
    _D(SERVO_FAIL_VIOLATION, 30086, "Switch servo mode failed because the robot safety mode is in violation", "suggest...") \
    _D(SERVO_FAIL_SYSTEMEMERGENCYSTOP, 30087, "Switch servo mode failed because the robot safety mode is in system emergency stop", "suggest...") \
    _D(SERVO_FAIL_ROBOTEMERGENCYSTOP, 30088, "Switch servo mode failed because the robot safety mode is in robot emergency stop", "Pop up the red emergency stop button on the teach pendant when the robot is in a safe range of motion") \
    _D(SERVO_FAIL_FAULT, 30089, "Switch servo mode failed because the robot safety mode is in fault", "suggest...") \
    _D(FREEDRIVE_FAIL_NO_RUNNING, 30090, "Enable handguide mode failed because the robot mode type is " _PH1_ "(not running)", "suggest...") \
    _D(RUNTIME_MACHINE_ERROR, 30091, "The state of the running machine is " _PH1_ ", not " _PH2_ ". " _PH3_ " function execution failed because the state is wrong." , "suggest...") \
    _D(RESUME_FAR_PAUSE_PT, 30092, "Cannot resume from joint position [" _PH1_ "]. Too far away from paused point [" _PH2_ "]." , "suggest...")\
    _D(PAYLOAD_ERROR, 30093, "The payload setting is too " _PH1_ "!" , "suggest...")

// clang-format on

#endif // AUBO_SDK_RTM_ERROR_H
