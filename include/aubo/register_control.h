#ifndef AUBO_SDK_REGISTER_CONTROL_INTERFACE_H
#define AUBO_SDK_REGISTER_CONTROL_INTERFACE_H

#include <stdint.h>
#include <memory>
#include <vector>

#include <aubo/type_def.h>
#include <aubo/global_config.h>

namespace arcs {
namespace common_interface {

/**
 * 通用寄存器
 */
class ARCS_ABI_EXPORT RegisterControl
{
public:
    RegisterControl();
    virtual ~RegisterControl();

    /**
     * Reads the boolean from one of the input registers, which can also be
     * accessed by a Field bus. Note, uses it’s own memory space.
     *
     * @param address: Address of the register (0:127)
     * @return The boolean value held by the register (true, false)
     *
     * @note he lower range of the boolean input registers [0:63] is reserved
     * for FieldBus/PLC interface usage. The upper range [64:127] cannot be
     * accessed by FieldBus/PLC interfaces, since it is reserved for external
     * RTDE clients.
     *
     * @code Python函数原型
     * getBoolInput(self: pyaubo_sdk.RegisterControl, arg0: int) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * getBoolInput(address: number) -> boolean
     * @endcode
     */
    bool getBoolInput(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @note 只有在实现 RTDE/Modbus Slave/PLC 服务端时使用
     *
     * @code Python函数原型
     * setBoolInput(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: bool) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * setBoolInput(address: number, value: boolean) -> nil
     * @endcode
     */
    int setBoolInput(uint32_t address, bool value);

    /**
     * Reads the integer from one of the input registers, which can also be
     * accessed by a Field bus. Note, uses it’s own memory space.
     *
     * @param address: Address of the register (0:47)
     * @return The value held by the register [-2,147,483,648 : 2,147,483,647]
     *
     * @note The lower range of the integer input registers [0:23] is reserved
     * for FieldBus/PLC interface usage. The upper range [24:47] cannot be
     * accessed by FieldBus/PLC interfaces, since it is reserved for external
     * RTDE clients.
     *
     * @code Python函数原型
     * getInt32Input(self: pyaubo_sdk.RegisterControl, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getInt32Input(address: number) -> number
     * @endcode
     */
    int getInt32Input(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @note 只有在实现 RTDE/Modbus Slave/PLC 服务端时使用
     *
     * @code Python函数原型
     * setInt32Input(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: int) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * setInt32Input(address: number, value: number) -> nil
     * @endcode
     */
    int setInt32Input(uint32_t address, int value);

    /**
     * Reads the float from one of the input registers, which can also be
     * accessed by a Field bus. Note, uses it’s own memory space.
     *
     * @param address: Address of the register (0:47)
     * @return The value held by the register (float)
     *
     * @note: The lower range of the float input registers [0:23] is reserved
     * for FieldBus/PLC interface usage. The upper range [24:47] cannot be
     * accessed by FieldBus/PLC interfaces, since it is reserved for external
     * RTDE clients.
     *
     * @code Python函数原型
     * getFloatInput(self: pyaubo_sdk.RegisterControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getFloatInput(address: number) -> number
     * @endcode
     */
    float getFloatInput(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @note 只有在实现 RTDE/Modbus Slave/PLC 服务端时使用
     *
     * @code Python函数原型
     * setFloatInput(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: float)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setFloatInput(address: number, value: number) -> nil
     * @endcode
     */
    int setFloatInput(uint32_t address, float value);

    /**
     *
     * @param address
     * @return
     *
     * @code Python函数原型
     * getDoubleInput(self: pyaubo_sdk.RegisterControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getDoubleInput(address: number) -> number
     * @endcode
     */
    double getDoubleInput(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @note 只有在实现 RTDE/Modbus Slave/PLC 服务端时使用
     *
     * @code Python函数原型
     * setDoubleInput(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: float)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setDoubleInput(address: number, value: number) -> nil
     * @endcode
     */
    int setDoubleInput(uint32_t address, double value);

    /**
     * Reads the boolean from one of the output registers, which can also be
     * accessed by a Field bus.
     * Note, uses it’s own memory space.
     *
     * @param address: Address of the register (0:127)
     * @return The boolean value held by the register (true, false)
     *
     * @note The lower range of the boolean output registers [0:63] is reserved
     * for FieldBus/PLC interface usage. The upper range [64:127] cannot be
     * accessed by FieldBus/PLC interfaces, since it is reserved for external
     * RTDE clients
     *
     * @code Python函数原型
     * getBoolOutput(self: pyaubo_sdk.RegisterControl, arg0: int) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * getBoolOutput(address: number) -> boolean
     * @endcode
     */
    bool getBoolOutput(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @code Python函数原型
     * setBoolOutput(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: bool) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * setBoolOutput(address: number, value: boolean) -> nil
     * @endcode
     */
    int setBoolOutput(uint32_t address, bool value);

    /**
     * Reads the integer from one of the output registers, which can also be
     * accessed by a Field bus. Note, uses it’s own memory space.
     *
     * @param address: Address of the register (0:47)
     * @return The int value held by the register [-2,147,483,648 :
     * 2,147,483,647]
     *
     * @note The lower range of the integer output registers [0:23] is reserved
     * for FieldBus/PLC interface usage. The upper range [24:47] cannot be
     * accessed by FieldBus/PLC interfaces, since it is reserved for external
     * RTDE clients.
     *
     * @code Python函数原型
     * getInt32Output(self: pyaubo_sdk.RegisterControl, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getInt32Output(address: number) -> number
     * @endcode
     */
    int getInt32Output(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @code Python函数原型
     * setInt32Output(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: int) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * setInt32Output(address: number, value: number) -> nil
     * @endcode
     */
    int setInt32Output(uint32_t address, int value);

    /**
     * Reads the float from one of the output registers, which can also be
     * accessed by a Field bus. Note, uses it’s own memory space.
     *
     * @param address: Address of the register (0:47)
     * @return The value held by the register (float)
     *
     * @note The lower range of the float output registers [0:23] is reserved
     * for FieldBus/PLC interface usage. The upper range [24:47] cannot be
     * accessed by FieldBus/PLC interfaces, since it is reserved for external
     * RTDE clients.
     *
     * @code Python函数原型
     * getFloatOutput(self: pyaubo_sdk.RegisterControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getFloatOutput(address: number) -> number
     * @endcode
     */
    float getFloatOutput(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @code Python函数原型
     * setFloatOutput(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: float)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setFloatOutput(address: number, value: number) -> nil
     * @endcode
     */
    int setFloatOutput(uint32_t address, float value);

    /**
     *
     * @param address
     * @return
     *
     * @code Python函数原型
     * getDoubleOutput(self: pyaubo_sdk.RegisterControl, arg0: int) -> float
     * @endcode
     *
     * @code Lua函数原型
     * getDoubleOutput(address: number) -> number
     * @endcode
     */
    double getDoubleOutput(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @code Python函数原型
     * setDoubleOutput(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: float)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setDoubleOutput(address: number, value: number) -> nil
     * @endcode
     */
    int setDoubleOutput(uint32_t address, double value);

    /**
     * 用于 Modbus Slave
     *
     * @param address
     * @return
     *
     * @code Python函数原型
     * getInt16Register(self: pyaubo_sdk.RegisterControl, arg0: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getInt16Register(address: number) -> number
     * @endcode
     */
    int16_t getInt16Register(uint32_t address);

    /**
     *
     * @param address
     * @param value
     * @return
     *
     * @code Python函数原型
     * setInt16Register(self: pyaubo_sdk.RegisterControl, arg0: int, arg1: int)
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setInt16Register(address: number, value: number) -> nil
     * @endcode
     */
    int setInt16Register(uint32_t address, int16_t value);

    /**
     * 具名变量是否存在
     *
     * @param key 变量名
     * @return
     */
    bool hasNamedVariable(const std::string &key);

    /**
     * 获取具名变量的类型
     *
     * @param key
     * @return
     */
    std::string getNamedVariableType(const std::string &key);

    /**
     * 具名变量是否更新
     *
     * @param key
     * @param since
     * @return
     *
     * @code Python函数原型
     * variableUpdated(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: int)
     * -> bool
     * @endcode
     *
     * @code Lua函数原型
     * variableUpdated(key: string, since: number) -> boolean
     * @endcode
     */
    bool variableUpdated(const std::string &key, uint64_t since);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getBool(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: bool) -> bool
     * @endcode
     *
     * @code Lua函数原型
     * getBool(key: string, default_value: boolean) -> boolean
     * @endcode
     */
    bool getBool(const std::string &key, bool default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setBool(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setBool(key: string, value: boolean) -> nil
     * @endcode
     */
    int setBool(const std::string &key, bool value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getVecChar(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: List[str])
     * -> List[str]
     * @endcode
     *
     * @code Lua函数原型
     * getVecChar(key: string, default_value: table) -> table
     * @endcode
     */
    std::vector<char> getVecChar(const std::string &key,
                                 const std::vector<char> &default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setVecChar(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: List[str])
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setVecChar(key: string, value: table) -> nil
     * @endcode
     */
    int setVecChar(const std::string &key, const std::vector<char> &value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getInt32(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * getInt32(key: string, default_value: number) -> number
     * @endcode
     */
    int getInt32(const std::string &key, int default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setInt32(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setInt32(key: string, value: number) -> nil
     * @endcode
     */
    int setInt32(const std::string &key, int value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getVecInt32(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: List[int])
     * -> List[int]
     * @endcode
     *
     * @code Lua函数原型
     * getVecInt32(key: string, default_value: table) -> table
     * @endcode
     */
    std::vector<int32_t> getVecInt32(const std::string &key,
                                     const std::vector<int32_t> &default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setVecInt32(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: List[int])
     * -> int
     * @endcode
     *
     * @code Lua函数原型
     * setVecInt32(key: string, value: table) -> nil
     * @endcode
     */
    int setVecInt32(const std::string &key, const std::vector<int32_t> &value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getFloat(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: float) ->
     * float
     * @endcode
     *
     * @code Lua函数原型
     * getFloat(key: string, default_value: number) -> number
     * @endcode
     */
    float getFloat(const std::string &key, float default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setFloat(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: float) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setFloat(key: string, value: number) -> nil
     * @endcode
     */
    int setFloat(const std::string &key, float value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getVecFloat(self: pyaubo_sdk.RegisterControl, arg0: str, arg1:
     * List[float]) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getVecFloat(key: string, default_value: table) -> table
     * @endcode
     */
    std::vector<float> getVecFloat(const std::string &key,
                                   const std::vector<float> &default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setVecFloat(self: pyaubo_sdk.RegisterControl, arg0: str, arg1:
     * List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setVecFloat(key: string, value: table) -> nil
     * @endcode
     */
    int setVecFloat(const std::string &key, const std::vector<float> &value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getDouble(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: float) ->
     * float
     * @endcode
     *
     * @code Lua函数原型
     * getDouble(key: string, default_value: number) -> number
     * @endcode
     */
    double getDouble(const std::string &key, double default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setDouble(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: float) ->
     * int
     * @endcode
     *
     * @code Lua函数原型
     * setDouble(key: string, value: number) -> nil
     * @endcode
     */
    int setDouble(const std::string &key, double value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getVecDouble(self: pyaubo_sdk.RegisterControl, arg0: str, arg1:
     * List[float]) -> List[float]
     * @endcode
     *
     * @code Lua函数原型
     * getVecDouble(key: string, default_value: table) -> table
     * @endcode
     */
    std::vector<double> getVecDouble(const std::string &key,
                                     const std::vector<double> &default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setVecDouble(self: pyaubo_sdk.RegisterControl, arg0: str, arg1:
     * List[float]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setVecDouble(key: string, value: table) -> nil
     * @endcode
     */
    int setVecDouble(const std::string &key, const std::vector<double> &value);

    /**
     *
     * @param key
     * @param default_value
     * @return
     *
     * @code Python函数原型
     * getString(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: str) -> str
     * @endcode
     *
     * @code Lua函数原型
     * getString(key: string, default_value: string) -> string
     * @endcode
     */
    std::string getString(const std::string &key,
                          const std::string &default_value);

    /**
     *
     * @param key
     * @param value
     * @return
     *
     * @code Python函数原型
     * setString(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * setString(key: string, value: string) -> nil
     * @endcode
     */
    int setString(const std::string &key, const std::string &value);

    /**
     *
     * @param key
     * @return
     *
     * @code Python函数原型
     * clearNamedVariable(self: pyaubo_sdk.RegisterControl, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * clearNamedVariable(key: string) -> nil
     * @endcode
     */
    int clearNamedVariable(const std::string &key);

    /**
     * Adds a new modbus signal for the controller to supervise. Expects no
     * response.
     *
     * @param device_info is rtu
     * format.eg,"serial_port,baud,parity,data_bit,stop_bit"
     * (1)The serial_port argument specifies the name of the
     * serial port eg. On Linux ,"/dev/ttyS0" or "/dev/ttyUSB0". On Windows,
     * \\.\COM10".
     * (2)The baud argument specifies the baud rate of the communication, eg.
     * 9600, 19200, 57600, 115200, etc.
     * (3)parity:N for none,E for even,O for odd.
     * (4)data_bit:The data_bits argument specifies the number of bits of data,
     * the allowed values are 5, 6, 7 and 8.
     * (5)stop_bit:The stop_bits argument
     * specifies the bits of stop, the allowed values are 1 and 2.
     *
     * device_info is tcp format.eg,"ip address,port"
     * (1)The ip address parameter specifies the ip address of the server
     * (2)The port parameter specifies the port number that the server is
     * listening on.
     * @param slave_number: An integer normally not used and set to 255, but is
     * a free choice between 0 and 255.
     * @param signal_address: An integer specifying the address of the either
     * the coil or the register that this new signal should reflect. Consult
     * the configuration of the modbus unit for this information.
     * @param signal_type: An integer specifying the type of signal to add. 0 =
     * digital input, 1 = digital output, 2 = register input and 3 = register
     * output.
     * @param signal_name: A string uniquely identifying the signal. If a
     * string is supplied which is equal to an already added signal, the new
     * signal will replace the old one. The length of the string cannot exceed
     * 20 characters.
     * @param sequential_mode: Setting to True forces the modbus client to wait
     * for a response before sending the next request. This mode is required by
     * some fieldbus units (Optional).
     * @return
     *
     * @code Python函数原型
     * modbusAddSignal(self: pyaubo_sdk.RegisterControl, arg0: str, arg1: int,
     * arg2: int, arg3: int, arg4: str, arg5: bool) -> int
     * @endcode
     *
     * @code Lua函数原型
     * modbusAddSignal(device_info: string, slave_number: number,
     * signal_address: number, signal_type: number, signal_name: string,
     * sequential_mode: boolean) -> nil
     * @endcode
     */
    int modbusAddSignal(const std::string &device_info, int slave_number,
                        int signal_address, int signal_type,
                        const std::string &signal_name, bool sequential_mode);

    /**
     * Deletes the signal identified by the supplied signal name.
     *
     * @param signal_name: A string equal to the name of the signal that should
     * be deleted.
     * @return
     *
     * @code Python函数原型
     * modbusDeleteSignal(self: pyaubo_sdk.RegisterControl, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * modbusDeleteSignal(signal_name: string) -> nil
     * @endcode
     */
    int modbusDeleteSignal(const std::string &signal_name);

    /**
     * Delete all modbus signals
     *
     * @return
     */
    int modbusDeleteAllSignals();

    /**
     * Reads the current value of a specific signal.
     *
     * @param signal_name: A string equal to the name of the signal for which
     * the value should be gotten.
     * @return An integer or a boolean. For digital signals: 1 or 0. For
     * register signals: The register value expressed as an integer.If the
     * value is -1, it means the signal does not exist
     *
     * @code Python函数原型
     * modbusGetSignalStatus(self: pyaubo_sdk.RegisterControl, arg0: str) -> int
     * @endcode
     *
     * @code Lua函数原型
     * modbusGetSignalStatus(signal_name: string) -> nil
     * @endcode
     */
    int modbusGetSignalStatus(const std::string &signal_name);

    /**
     * 获取所有信号的名字集合
     *
     * @return
     */
    std::vector<std::string> modbusGetSignalNames();

    /**
     * 获取所有信号的类型集合
     *
     * @return
     */
    std::vector<int> modbusGetSignalTypes();

    /**
     * 获取所有信号的数值集合
     *
     * @return
     */
    std::vector<int> modbusGetSignalValues();

    /**
     * 获取所有信号的请求是否有错误(0:无错误,其他:有错误)集合
     *
     * @return
     */
    std::vector<int> modbusGetSignalErrors();

    /**
     * Sends a command specified by the user to the modbus unit located on the
     * specified IP address. Cannot be used to request data, since the response
     * will not be received. The user is responsible for supplying data which
     * is meaningful to the supplied function code. The builtin function takes
     * care of constructing the modbus frame, so the user should not be
     * concerned with the length of the command.
     *
     * @param device_info is rtu
     * format.eg,"serial_port,baud,parity,data_bit,stop_bit"
     * (1)The serial_port argument specifies the name of the
     * serial port eg. On Linux ,"/dev/ttyS0" or "/dev/ttyUSB0". On Windows,
     * \\.\COM10".
     * (2)The baud argument specifies the baud rate of the communication, eg.
     * 9600, 19200, 57600, 115200, etc.
     * (3)parity:N for none,E for even,O for odd.
     * (4)data_bit:The data_bits argument specifies the number of bits of data,
     * the allowed values are 5, 6, 7 and 8.
     * (5)stop_bit:The stop_bits argument
     * specifies the bits of stop, the allowed values are 1 and 2.
     *
     * device_info is tcp format.eg,"ip address,port"
     * @param slave_number: An integer specifying the slave number to use for
     * the custom command.
     * @param function_code: An integer specifying the function code for the
     * custom command.
     * @param data: An array of integers in which each entry must be a valid
     * byte (0-255) value.
     * @return
     *
     * @code Python函数原型
     * modbusSendCustomCommand(self: pyaubo_sdk.RegisterControl, arg0: str,
     * arg1: int, arg2: int, arg3: List[int]) -> int
     * @endcode
     *
     * @code Lua函数原型
     * modbusSendCustomCommand(device_info: string, slave_number: number,
     * function_code: number, data: table) -> nil
     * @endcode
     */
    int modbusSendCustomCommand(const std::string &device_info,
                                int slave_number, int function_code,
                                const std::vector<uint8_t> &data);

    /**
     * Sets the selected digital input signal to either a "default" or
     * "freedrive" action.
     *
     * @param robot_name: A string identifying a robot name that conncted robot
     * @param signal_name: A string identifying a digital input signal that was
     * previously added.
     * @param action: The type of action. The action can either be "default" or
     * "freedrive". (string)
     * @return
     *
     * @code Python函数原型
     * modbusSetDigitalInputAction(self: pyaubo_sdk.RegisterControl, arg0: str,
     * arg1: str, arg2: int)
     * @endcode
     *
     * @code Lua函数原型
     * modbusSetDigitalInputAction(robot_name: string, signal_name: string,
     * action: number) -> nil
     * @endcode
     */
    int modbusSetDigitalInputAction(const std::string &robot_name,
                                    const std::string &signal_name,
                                    StandardInputAction action);

    /**
     * 设置 Modbus 信号输出动作
     *
     * @param robot_name
     * @param signal_name
     * @param runstate
     * @return
     */
    int modbusSetOutputRunstate(const std::string &robot_name,
                                const std::string &signal_name,
                                StandardOutputRunState runstate);

    /**
     * Sets the output register signal identified by the given name to the
     * given value.
     *
     * @param signal_name: A string identifying an output register signal that
     * in advance has been added.
     * @param value: An integer which must be a valid word (0-65535)
     * value.
     * @return
     *
     * @code Python函数原型
     * modbusSetOutputSignal(self: pyaubo_sdk.RegisterControl, arg0: str, arg1:
     * int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * modbusSetOutputSignal(signal_name: string, value: number) -> nil
     * @endcode
     */
    int modbusSetOutputSignal(const std::string &signal_name, uint16_t value);

    /**
     * Sets the frequency with which the robot will send requests to the Modbus
     * controller to either read or write the signal value.
     *
     * @param signal_name: A string identifying an output digital signal that
     * in advance has been added.
     * @param update_frequency: An integer in the range 0-125 specifying the
     * update frequency in Hz.
     * @return
     *
     * @code Python函数原型
     * modbusSetSignalUpdateFrequency(self: pyaubo_sdk.RegisterControl, arg0:
     * str, arg1: int) -> int
     * @endcode
     *
     * @code Lua函数原型
     * modbusSetSignalUpdateFrequency(signal_name: string, update_frequency:
     * number) -> nil
     * @endcode
     */
    int modbusSetSignalUpdateFrequency(const std::string &signal_name,
                                       int update_frequency);

protected:
    void *d_;
};
using RegisterControlPtr = std::shared_ptr<RegisterControl>;

// clang-format off
#define RegisterControl_DECLARES                                \
    _FUNC(RegisterControl, 1, getBoolInput, address)            \
    _INST(RegisterControl, 2, setBoolInput, address, value)     \
    _FUNC(RegisterControl, 1, getInt32Input, address)           \
    _INST(RegisterControl, 2, setInt32Input, address, value)    \
    _FUNC(RegisterControl, 1, getFloatInput, address)           \
    _INST(RegisterControl, 2, setFloatInput, address, value)    \
    _FUNC(RegisterControl, 1, getDoubleInput, address)          \
    _INST(RegisterControl, 2, setDoubleInput, address, value)   \
    _FUNC(RegisterControl, 1, getBoolOutput, address)           \
    _INST(RegisterControl, 2, setBoolOutput, address, value)    \
    _FUNC(RegisterControl, 1, getInt32Output, address)          \
    _INST(RegisterControl, 2, setInt32Output, address, value)   \
    _FUNC(RegisterControl, 1, getFloatOutput, address)          \
    _INST(RegisterControl, 2, setFloatOutput, address, value)   \
    _FUNC(RegisterControl, 1, getDoubleOutput, address)         \
    _INST(RegisterControl, 2, setDoubleOutput, address, value)  \
    _FUNC(RegisterControl, 1, getInt16Register, address)        \
    _INST(RegisterControl, 2, setInt16Register, address, value) \
    _FUNC(RegisterControl, 2, variableUpdated, key, since)      \
    _FUNC(RegisterControl, 1, hasNamedVariable, key)            \
    _FUNC(RegisterControl, 1, getNamedVariableType, key)        \
    _FUNC(RegisterControl, 2, getBool, key, default_value)      \
    _INST(RegisterControl, 2, setBool, key, value)              \
    _FUNC(RegisterControl, 2, getVecChar, key, default_value)   \
    _INST(RegisterControl, 2, setVecChar, key, value)           \
    _FUNC(RegisterControl, 2, getInt32, key, default_value)     \
    _INST(RegisterControl, 2, setInt32, key, value)             \
    _FUNC(RegisterControl, 2, getVecInt32, key, default_value)  \
    _INST(RegisterControl, 2, setVecInt32, key, value)          \
    _FUNC(RegisterControl, 2, getFloat, key, default_value)     \
    _INST(RegisterControl, 2, setFloat, key, value)             \
    _FUNC(RegisterControl, 2, getVecFloat, key, default_value)  \
    _INST(RegisterControl, 2, setVecFloat, key, value)          \
    _FUNC(RegisterControl, 2, getDouble, key, default_value)    \
    _INST(RegisterControl, 2, setDouble, key, value)            \
    _FUNC(RegisterControl, 2, getVecDouble, key, default_value) \
    _INST(RegisterControl, 2, setVecDouble, key, value)         \
    _FUNC(RegisterControl, 2, getString, key, default_value)    \
    _INST(RegisterControl, 2, setString, key, value)            \
    _INST(RegisterControl, 1, clearNamedVariable, key)          \
    _INST(RegisterControl, 6, modbusAddSignal, device_info, slave_number, signal_address, signal_type, signal_name, sequential_mode) \
    _INST(RegisterControl, 1, modbusDeleteSignal, signal_name) \
    _FUNC(RegisterControl, 0, modbusDeleteAllSignals)          \
    _FUNC(RegisterControl, 1, modbusGetSignalStatus, signal_name) \
    _FUNC(RegisterControl, 0, modbusGetSignalNames)            \
    _FUNC(RegisterControl, 0, modbusGetSignalTypes)            \
    _FUNC(RegisterControl, 0, modbusGetSignalValues)           \
    _FUNC(RegisterControl, 0, modbusGetSignalErrors)           \
    _INST(RegisterControl, 4, modbusSendCustomCommand, IP, slave_number, function_code, data) \
    _INST(RegisterControl, 3, modbusSetDigitalInputAction, robot_name, signal_name, action) \
    _INST(RegisterControl, 3, modbusSetOutputRunstate, robot_name, signal_name, runstate)   \
    _INST(RegisterControl, 2, modbusSetOutputSignal, signal_name, value) \
    _INST(RegisterControl, 2, modbusSetSignalUpdateFrequency, signal_name, update_frequency)
// clang-format on
} // namespace common_interface
} // namespace arcs

#endif // AUBO_SDK_REGISTER_CONTROL_INTERFACE_H
