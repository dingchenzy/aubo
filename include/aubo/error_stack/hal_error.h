#ifndef AUBO_SDK_HAL_ERROR_H
#define AUBO_SDK_HAL_ERROR_H

// 缩写说明
// JNT: joint
// PDL: pedstral
// TP: teach pendant
// COMM: communication
// ENC: encoder
// CURR: current
// POS: position
// PKG: package
// PROG: program

//#define _D(n, x, y, s)  _D(JOINT_##n, codeCompose(1, x, y), s)
#define _TOOL_D(n, x, y, s, r) _D(TOOL_##n, codeCompose(2, x, y), s, r)
#define _PDL_D(n, x, y, s, r)  _D(PEDSTRAL_##n, codeCompose(3, x, y), s, r)
#define _SIB_D(n, x, y, s, r)  _D(SIB_##n, codeCompose(4, x, y), s, r)
#define _HW_D(n, x, y, s, r)   _D(HW_##n, codeCompose(5, x, y), s, r)

// clang-format off
#define JOINT_ERRORS \
    _D(JOINT_ERR_OVER_CURRENET,  10001, "joint" _PH1_ " error: over current", "(a) Check for short circuit. (b) Do a Complete rebooting sequence. (c) If this happens more than two times in a row, replace joint") \
    _D(JOINT_ERR_OVER_VOLTAGE,  10002, "joint" _PH1_ " error: over voltage", "(a) Do a Complete rebooting sequence. (b) Check 48 V Power supply, current distributer, energy eater and Control Board for issues") \
    _D(JOINT_ERR_LOW_VOLTAGE,  10003, "joint" _PH1_ " error: low voltage", "(a) Do a Complete rebooting sequence. (b) Check for short circuit in robot arm. (c) Check 48 V Power supply, current distributer, energy eater and Control Board for issues") \
    _D(JOINT_ERR_OVER_TEMP,  10004, "joint" _PH1_ " error: over temperature", "(a) Check robot’s environment and make sure the robot is operating within recommended limits. (b) Do a Complete rebooting sequence") \
    _D(JOINT_ERR_HALL,  10005, "joint" _PH1_ " error: hall", "suggest...") \
    _D(JOINT_ERR_ENCODER,  10006, "joint" _PH1_ " error: encoder", "Check encoder connections") \
    _D(JOINT_ERR_ABS_ENCODER,  10007, "joint" _PH1_ " error: abs encoder", "suggest...") \
    _D(JOINT_ERR_Q_CURRENT,  10008, "joint" _PH1_ " error: detect current", "suggest...") \
    _D(JOINT_ERR_ENC_POLL,  10009, "joint" _PH1_ " error: encoder pollustion", "suggest...") \
    _D(JOINT_ERR_ENC_Z_SIGNAL,  10010, "joint" _PH1_ " error: enocder z signal", "suggest...") \
    _D(JOINT_ERR_ENC_CAL,  10011, "joint" _PH1_ " error: encoder calibrate", "suggest...") \
    _D(JOINT_ERR_IMU_SENS, 10012, "joint" _PH1_ " error: IMU sensor", "suggest...") \
    _D(JOINT_ERR_TEMP_SENS, 10013, "joint" _PH1_ " error: TEMP sensor", "suggest...") \
    _D(JOINT_ERR_CAN_BUS, 10014, "joint" _PH1_ " error: can bus error", "suggest...") \
    _D(JOINT_ERR_SYS_CUR, 10015, "joint" _PH1_ " error: system current error", "suggest...") \
    _D(JOINT_ERR_SYS_POS, 10016, "joint" _PH1_ " error: system position error","suggest...") \
    _D(JOINT_ERR_OVER_SP, 10017, "joint" _PH1_ " error: over speed","suggest...") \
    _D(JOINT_ERR_OVER_ACC, 10018, "joint" _PH1_ " error: over accelerate", "suggest...") \
    _D(JOINT_ERR_TRACE, 10019, "joint" _PH1_ " error: trace accuracy", "suggest...") \
    _D(JOINT_ERR_TAG_POS_OVER, 10020, "joint" _PH1_ " error: target position out of range", "suggest...") \
    _D(JOINT_ERR_TAG_SP_OVER, 10021, "joint" _PH1_ " error: target speed out of range", "suggest...") \
    _D(JOINT_ERR_COLLISION, 10022, "joint" _PH1_ " error: collision", "suggest...")

#define TOOL_ERRORS \
    _TOOL_D(FLASH_VERIFY_FAILED, 1, 1, "Flash write verify failed", "suggest...") \
    _TOOL_D(PROGRAM_CRC_FAILED, 1, 2, "Program flash checksum failed during bootloading", "suggest...") \
    _TOOL_D(PROGRAM_CRC_FAILED2, 1, 3, "Program flash checksum failed at runtime", "suggest...") \
    _TOOL_D(ID_UNDIFINED, 1, 4, "Tool ID is undefined", "suggest...") \
    _TOOL_D(ILLEGAL_BL_CMD, 1, 5, "Illegal bootloader command", "suggest...") \
    _TOOL_D(FW_WRONG, 1, 6, "Wrong firmware at the joint", "suggest...") \
    _TOOL_D(HW_INVALID, 1, 7, "Invalid hardware revision", "suggest...") \
    _TOOL_D(SHORT_CURCUIT_H, 2, 1, "Short circuit detected on Digital Output: " _PH1_ " high side", "suggest...") \
    _TOOL_D(SHORT_CURCUIT_L, 2, 2, "Short circuit detected on Digital Output: " _PH1_ " low side", "suggest...") \
    _TOOL_D(AVERAGE_CURR_HIGH, 2, 3, "10 second Average tool IO Current of " _PH1_ " A is outside of the allowed range.", "suggest...") \
    _TOOL_D(POWER_PIN_OVER_CURR, 2, 4, "Current of " _PH1_ " A on the POWER pin is outside of the allowed range.", "suggest...") \
    _TOOL_D(DOUT_PIN_OVER_CURR, 2, 5, "Current of " _PH1_ " A on the Digital Output pins is outside of the allowed range.", "suggest...") \
    _TOOL_D(GROUND_PIN_OVER_CURR, 2, 6, "Current of " _PH1_ " A on the ground pin is outside of the allowed range.", "suggest...") \
    _TOOL_D(RX_FRAMING, 3, 1, "RX framing error", "suggest...") \
    _TOOL_D(RX_PARITY, 3, 2, "RX Parity error", "suggest...") \
    _TOOL_D(48V_LOW, 4, 1, "48V input is too low", "suggest...") \
    _TOOL_D(48V_HIGH, 4, 2, "48V input is too high", "suggest...")

#define PEDSTRAL_ERRORS \
    _PDL_D(PKG_LOST, 1, 1, "Lost package from pedstral", "suggest...")

#define SAFETY_INTERFACE_BOARD_ERRORS \
    _D(IFB_ERR_ROBOTTYPE, 20001, "robot error type! 机械臂类型错误", "suggest...") \
    _D(IFB_ERR_ADXL_SENS, 20002, "adxl sensor error! 加速度计芯片错误", "suggest...") \
    _D(IFB_ERR_EN_LINE, 20003, "encoder line error! 编码器线数错误", "suggest...") \
    _D(IFB_ERR_ENTER_HDG_MODE, 20004, "robot enter hdg mode! 进入拖动示教模式错误", "suggest...") \
    _D(IFB_ERR_EXIT_HDG_MODE, 20005, "robot exit hdg mode! 退出拖动示教模式错误", "suggest...") \
    _D(IFB_ERR_MAC_DATA_BREAK, 20006, "mac data break! MAC数据中断错误", "suggest...") \
    _D(IFB_ERR_DRV_FIRMWARE_VERSION, 20007, "driver firmware version error! 驱动器版本错误（关节固件版本不一致）", "suggest...") \
    _D(INIT_ERR_EN_DRV, 20008, "driver enable failed! 机械臂初始化使能驱动器失败", "suggest...") \
    _D(INIT_ERR_EN_AUTO_BACK, 20009, "driver enable auto back failed! 机械臂初始化使能自动回应失败", "suggest...") \
    _D(INIT_ERR_EN_CUR_LOOP, 20010, "driver enable current loop failed! 机械臂初始化使能电流环失败", "suggest...") \
    _D(INIT_ERR_SET_TAG_CUR, 20011, "driver set target current failed! 机械臂初始化设置目标电流失败", "suggest...") \
    _D(INIT_ERR_RELEASE_BRAKE, 20012, "driver release brake failed! 机械臂初始化释放刹车失败", "suggest...") \
    _D(INIT_ERR_EN_POS_LOOP, 20013, "driver enable postion loop failed! 机械臂初始化使能位置环失败", "suggest...") \
    _D(INIT_ERR_SET_MAX_ACC, 20014, "set max accelerate failed! 机械臂初始化设置最大加速度失败", "suggest...") \
    _D(SAFETY_ERR_PROTECTION_STOP_TIMEOUT, 20015, "protection stop timeout! 机械臂保护停止超时", "suggest...") \
    _D(SAFETY_ERR_REDUCED_MODE_TIMEOUT, 20016, "reduced mode timeout! 机械臂减速模式超时", "suggest...") \
    _D(SYS_ERR_MCU_COM, 20017, "robot system error:mcu communication error!", "suggest...") \
    _D(SYS_ERR_RS485_COM, 20018, "robot system error:RS485 communication error!", "suggest...") \
    _D(IFB_ERR_DISCONNECTED, 20019, "Interface board may be disconnected. Please check connection between IPC and Interface board.", "suggest...")

#define HARDWARE_INTERFACE_ERRORS \
    _HW_D(SCB_SETUP_FAILED, 1, 1, "Setup of Safety Control Board failed", "suggest...") \
    _HW_D(PKG_CNT_DISAGEE, 1, 2, "Packet counter disagreements", "suggest...") \
    _HW_D(SCB_DISCONNECT, 1, 3, "Connection to Safety Control Board lost", "suggest...") \
    _HW_D(SCB_PKG_LOST, 1, 4, "Package lost from Safety Control Board", "suggest...") \
    _HW_D(SCB_CONN_INIT_FAILED, 1, 5, "Ethernet connection initialization with Safety Control Board failed", "suggest...") \
    _HW_D(LOST_JOINT_PKG, 1, 6, "Lost package from joint  " _PH1_ "", "suggest...") \
    _HW_D(LOST_TOOL_PKG, 1, 7, "Lost package from tool", "suggest...") \
    _HW_D(JOINT_PKG_CNT_DISAGREE, 1, 8, "Packet counter disagreement in packet from joint " _PH1_ "", "suggest...") \
    _HW_D(TOOL_PKG_CNT_DISAGREE, 1, 9, "Packet counter disagreement in packet from tool", "suggest...") \
    _HW_D(JOINTS_FAULT, 2, 1, "" _PH1_ " joint entered the Fault State", "suggest...") \
    _HW_D(JOINTS_VIOLATION, 2, 2, "" _PH1_ " joint entered the Violation State", "suggest...") \
    _HW_D(TP_FAULT, 2, 3, "Teach Pendant entered the Fault State", "suggest...") \
    _HW_D(TP_VIOLATION, 2, 4, "Teach Pendant entered the Violation State", "suggest...") \
    _HW_D(JOINT_MV_TOO_FAR, 3, 1, "" _PH1_ " joint moved too far before robot entered RUNNING State", "suggest...") \
    _HW_D(JOINT_STOP_NOT_FAST, 3, 2, "Joint Not stopping fast enough", "suggest...") \
    _HW_D(JOINT_MV_LIMIT, 3, 3, "Joint moved more than allowable limit", "suggest...") \
    _HW_D(FT_SENSOR_DATA_INVALID, 4, 1, "Force-Torque Sensor data invalid", "suggest...") \
    _HW_D(NO_FT_SENSOR, 4, 2, "Force-Torque sensor is expected, but it cannot be detected", "suggest...") \
    _HW_D(FT_SENSOR_NOT_CALIB, 4, 3, "Force-Torque sensor is detected but not calibrated", "suggest...") \
    _HW_D(RELEASE_BRAKE_FAILED, 5, 0, "Robot was not able to brake release, see log for details", "suggest...") \
    _HW_D(OVERCURR_SHUTDOWN, 6, 0, "Overcurrent shutdown", "suggest...") \
    _HW_D(ENERGEY_SURPLUS, 7, 0, "Energy surplus shutdown", "suggest...") \
    _HW_D(IDLE_POWER_HIGH, 8, 0, "Idle power consumption to high", "suggest...")

// clang-format on

// 定义硬件抽象层的错误代码
#define HAL_ERRORS                \
    JOINT_ERRORS                  \
    TOOL_ERRORS                   \
    PEDSTRAL_ERRORS               \
    SAFETY_INTERFACE_BOARD_ERRORS \
    HARDWARE_INTERFACE_ERRORS

#endif // AUBO_SDK_JOINT_ERROR_H
