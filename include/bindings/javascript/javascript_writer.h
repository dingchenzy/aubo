#ifndef JAVASCRIPT_WRITER_H
#define JAVASCRIPT_WRITER_H

#include <sstream>
#include <iomanip>
#include "bindings/function_traits.h"

class JavascriptWriter
{
public:
    JavascriptWriter(std::ostream &os) : os_(os) {}

#define INDENT std::setw(indent_ * 2) << ""
    JavascriptWriter &defineClass(const std::string &clazz)
    {
        clazz_ = clazz + ".";
        call_prefix_ = "      ";
        os_ << INDENT << "\nexport class " << clazz << " {" << std::endl;
        indent_++;
        return *this;
    }

    JavascriptWriter &endClass()
    {
        if (indent_ > 0) {
            indent_--;
        }
        os_ << INDENT << "}" << std::endl;
        clazz_ = "";
        robot_name_ = "";
        return *this;
    }

    JavascriptWriter &constructor()
    {
        robot_name_ = "";
        os_ << INDENT << "constructor(private rpcClient: Client) {"
            << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }
    JavascriptWriter &constructorForRobot()
    {
        robot_name_ = "";
        os_ << INDENT
            << "constructor(private rpcClient: Client, private robotName: "
               "string) {"
            << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }
    JavascriptWriter &constructorForRobotInterface()
    {
        os_ << INDENT << "private readonly rpcClient: Client;" << std::endl;
        os_ << INDENT << "private readonly config: RobotConfig;" << std::endl;
        os_ << INDENT << "private readonly motion: MotionControl;" << std::endl;
        os_ << INDENT << "private readonly force: ForceControl;" << std::endl;
        os_ << INDENT << "private readonly io: IoControl;" << std::endl;
        os_ << INDENT << "private readonly syncMove: SyncMove;" << std::endl;
        os_ << INDENT << "private readonly algorithm: RobotAlgorithm;"
            << std::endl;
        os_ << INDENT << "private readonly manage: RobotManage;" << std::endl;
        os_ << INDENT << "private readonly state: RobotState;" << std::endl;
        os_ << INDENT << "private readonly trace: Trace;\n" << std::endl;
        os_ << INDENT << "constructor(client: Client, robotName: string) {"
            << std::endl;
        os_ << INDENT << "  this.rpcClient = client;" << std::endl;
        os_ << INDENT << "  this.config = new RobotConfig(client, robotName);"
            << std::endl;
        os_ << INDENT << "  this.motion = new MotionControl(client, robotName);"
            << std::endl;
        os_ << INDENT << "  this.force = new ForceControl(client, robotName);"
            << std::endl;
        os_ << INDENT << "  this.io = new IoControl(client, robotName);"
            << std::endl;
        os_ << INDENT << "  this.syncMove = new SyncMove(client, robotName);"
            << std::endl;
        os_ << INDENT
            << "  this.algorithm = new RobotAlgorithm(client, robotName);"
            << std::endl;
        os_ << INDENT << "  this.manage = new RobotManage(client, robotName);"
            << std::endl;
        os_ << INDENT << "  this.state = new RobotState(client, robotName);"
            << std::endl;
        os_ << INDENT << "  this.trace = new Trace(client, robotName);"
            << std::endl;
        os_ << INDENT << "}\n" << std::endl;

        return *this;
    }
    JavascriptWriter &constructorForAuboApi()
    {
        os_ << INDENT << "private robotName: string | undefined;" << std::endl;
        os_ << INDENT << "private _client: Client | undefined;" << std::endl;
        os_ << INDENT << "private _systemInfo: SystemInfo | undefined;"
            << std::endl;
        os_ << INDENT << "private _runtimeMachine: RuntimeMachine | undefined;"
            << std::endl;
        os_ << INDENT
            << "private _registerControl: RegisterControl | undefined;"
            << std::endl;
        os_ << INDENT << "private _math: Math | undefined;" << std::endl;
        os_ << INDENT << "private _serial: Serial | undefined;" << std::endl;
        os_ << INDENT << "private _socket: Socket | undefined;" << std::endl;
        os_ << INDENT
            << "private _robotInterfaces: Map<string, RobotInterface> = new "
               "Map<string,RobotInterface>();\n"
            << std::endl;
        os_ << INDENT << "constructor(client?: Client) {" << std::endl;
        os_ << INDENT << "  if (client) this.init(client);" << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        os_ << INDENT << "init(client: Client) {" << std::endl;
        os_ << INDENT << "  this._client = client;" << std::endl;
        os_ << INDENT << "  this._systemInfo = new SystemInfo(client);"
            << std::endl;
        os_ << INDENT << "  this._runtimeMachine = new RuntimeMachine(client);"
            << std::endl;
        os_ << INDENT << "  this._math = new Math(client);" << std::endl;
        os_ << INDENT << "  this._serial = new Serial(client);" << std::endl;
        os_ << INDENT << "  this._socket = new Socket(client);" << std::endl;
        os_ << INDENT
            << "  this._registerControl = new RegisterControl(client);"
            << std::endl;
        os_ << INDENT << "}\n" << std::endl;

        return *this;
    }

    JavascriptWriter &defineFunction(const std::string &func,
                                     const std::string &ret_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "()";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "'\n    );" << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }

    JavascriptWriter &defineFunction(const std::string &func,
                                     const std::string &ret_t,
                                     const std::string &arg1,
                                     const std::string &arg1_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "(" + arg1 + ": " + arg1_t + ")";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "',\n      [" << arg1 << "]\n    );" << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }

    JavascriptWriter &defineFunction(const std::string &func,
                                     const std::string &ret_t,
                                     const std::string &arg1,
                                     const std::string &arg1_t,
                                     const std::string &arg2,
                                     const std::string &arg2_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "(\n    " + arg1 + ": " + arg1_t + ",\n    " + arg2 + ": " +
                    arg2_t + "\n  )";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "',\n      [" << arg1 << ", " << arg2 << "]\n    );"
            << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }

    JavascriptWriter &defineFunction(
        const std::string &func, const std::string &ret_t,
        const std::string &arg1, const std::string &arg1_t,
        const std::string &arg2, const std::string &arg2_t,
        const std::string &arg3, const std::string &arg3_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "(\n    " + arg1 + ": " + arg1_t + ",\n    " + arg2 + ": " +
                    arg2_t + ",\n    " + arg3 + ": " + arg3_t + "\n  )";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "',\n      [" << arg1 << ", " << arg2 << ", " << arg3
            << "]\n    );" << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }

    JavascriptWriter &defineFunction(
        const std::string &func, const std::string &ret_t,
        const std::string &arg1, const std::string &arg1_t,
        const std::string &arg2, const std::string &arg2_t,
        const std::string &arg3, const std::string &arg3_t,
        const std::string &arg4, const std::string &arg4_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "(\n    " + arg1 + ": " + arg1_t + ",\n    " + arg2 + ": " +
                    arg2_t + ",\n    " + arg3 + ": " + arg3_t + ",\n    " +
                    arg4 + ": " + arg4_t + "\n  )";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "',\n      [" << arg1 << ", " << arg2 << ", " << arg3 << ", "
            << arg4 << "]\n    );" << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }

    JavascriptWriter &defineFunction(
        const std::string &func, const std::string &ret_t,
        const std::string &arg1, const std::string &arg1_t,
        const std::string &arg2, const std::string &arg2_t,
        const std::string &arg3, const std::string &arg3_t,
        const std::string &arg4, const std::string &arg4_t,
        const std::string &arg5, const std::string &arg5_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "(\n    " + arg1 + ": " + arg1_t + ",\n    " + arg2 + ": " +
                    arg2_t + ",\n    " + arg3 + ": " + arg3_t + ",\n    " +
                    arg4 + ": " + arg4_t + ",\n    " + arg5 + ": " + arg5_t +
                    "\n  )";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "',\n      [" << arg1 << ", " << arg2 << ", " << arg3 << ", "
            << arg4 << ", " << arg5 << "]\n    );" << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }

    JavascriptWriter &defineFunction(
        const std::string &func, const std::string &ret_t,
        const std::string &arg1, const std::string &arg1_t,
        const std::string &arg2, const std::string &arg2_t,
        const std::string &arg3, const std::string &arg3_t,
        const std::string &arg4, const std::string &arg4_t,
        const std::string &arg5, const std::string &arg5_t,
        const std::string &arg6, const std::string &arg6_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "(\n    " + arg1 + ": " + arg1_t + ",\n    " + arg2 + ": " +
                    arg2_t + ",\n    " + arg3 + ": " + arg3_t + ",\n    " +
                    arg4 + ": " + arg4_t + ",\n    " + arg5 + ": " + arg5_t +
                    ",\n    " + arg6 + ": " + arg6_t + "\n  )";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "',\n      [" << arg1 << ", " << arg2 << ", " << arg3 << ", "
            << arg4 << ", " << arg5 << ", " << arg6 << "]\n    );" << std::endl;
        os_ << INDENT << "}\n" << std::endl;
        return *this;
    }

    JavascriptWriter &defineFunction(
        const std::string &func, const std::string &ret_t,
        const std::string &arg1, const std::string &arg1_t,
        const std::string &arg2, const std::string &arg2_t,
        const std::string &arg3, const std::string &arg3_t,
        const std::string &arg4, const std::string &arg4_t,
        const std::string &arg5, const std::string &arg5_t,
        const std::string &arg6, const std::string &arg6_t,
        const std::string &arg7, const std::string &arg7_t,
        const std::string &arg8, const std::string &arg8_t,
        const std::string &arg9, const std::string &arg9_t,
        const std::string &arg10, const std::string &arg10_t)
    {
        return_type_ = "Promise<unknown | " + ret_t + ">";
        arg_type_ = "(\n    " + arg1 + ": " + arg1_t + ",\n    " + arg2 + ": " +
                    arg2_t + ",\n    " + arg3 + ": " + arg3_t + ",\n    " +
                    arg4 + ": " + arg4_t + ",\n    " + arg5 + ": " + arg5_t +
                    ",\n    " + arg6 + ": " + arg6_t + ",\n    " + arg7 + ": " +
                    arg7_t + ",\n    " + arg8 + ": " + arg8_t + ",\n    " +
                    arg9 + ": " + arg9_t + ",\n    " + arg10 + ": " + arg10_t +
                    "\n  )";
        os_ << INDENT << "public async " << func << arg_type_ << ": "
            << return_type_ << " {" << std::endl;
        os_ << INDENT << "  return this.rpcClient.call(\n"
            << call_prefix_ << "'" << robot_name_ << clazz_ << func
            << "',\n      [" << arg1 << ", " << arg2 << ", " << arg3 << ", "
            << arg4 << ", " << arg5 << ", " << arg6 << ", " << arg7 << ", "
            << arg8 << ", " << arg9 << ", " << arg10 << "]\n    );"
            << std::endl;
        os_ << INDENT << "}" << std::endl;
        return *this;
    }

    JavascriptWriter &defineRobotInterfaceMember(const std::string &func,
                                                 const std::string &ret)
    {
        os_ << INDENT << "public " << func
            << "():" << func.substr(3, func.length()) << " {" << std::endl;
        os_ << INDENT << "  return this." << ret << std::endl;
        os_ << INDENT << "}" << std::endl;
        return *this;
    }

    JavascriptWriter &defineAuboApiMember(const std::string &func,
                                          const std::string &ret)
    {
        os_ << INDENT << "public " << func
            << "():" << func.substr(3, func.length()) << " {" << std::endl;
        os_ << INDENT << "  return <" << func.substr(3, func.length())
            << ">this." << ret << std::endl;
        os_ << INDENT << "}" << std::endl;
        return *this;
    }

    JavascriptWriter &defineImport(const std::string &arg1,
                                   const std::string &arg2)
    {
        os_ << "import { " << arg1 << " } from " << '"' << arg2 << '"'
            << std::endl;
        return *this;
    }
    JavascriptWriter &writeLine(const std::string &content)
    {
        os_ << INDENT << content << std::endl;
        return *this;
    }
    JavascriptWriter &defineTuple(const std::string &name,
                                  const std::string &elements)
    {
        os_ << "export type " << name << " = [" << elements << "];"
            << std::endl;
        return *this;
    }
    JavascriptWriter &defineMap(const std::string &name,
                                const std::string &elements)
    {
        os_ << "export type " << name << " = Map<" << elements << ">;"
            << std::endl;
        return *this;
    }
    JavascriptWriter &defineSet(const std::string &name,
                                const std::string &elements)
    {
        os_ << "export type " << name << " = Set<" << elements << ">;"
            << std::endl;
        return *this;
    }
    JavascriptWriter &defineEnumHead(const std::string &name)
    {
        os_ << "export enum " << name << " {" << std::endl;
        return *this;
    }
    JavascriptWriter &defineEnumEnd()
    {
        os_ << "}\n" << std::endl;
        return *this;
    }
    JavascriptWriter &defineIndexExport(const std::string &filename)
    {
        os_ << "export * from './" << filename << "';" << std::endl;
        return *this;
    }

    std::ostream &os() { return os_; }

private:
    std::ostream &os_;
    int indent_ = 0;
    std::string clazz_{ "" };
    std::string return_type_{ "" };
    std::string arg_type_{ "" };

public:
    std::string robot_name_{ "" };
    std::string call_prefix_{ "    " };
};

#define TRANS_TPYE(CPP_TYPE, JS_TYPE)            \
    if constexpr (std::is_same<T, CPP_TYPE>()) { \
        return #JS_TYPE;                         \
    }

using PlanContextTuple = std::tuple<int, int, std::string>;
using WorkObjectHoldTuple = std::tuple<std::string, std::vector<double>>;
using FirmwareUpdateProcessTuple = std::tuple<std::string, double>;
using DHParam = std::unordered_map<std::string, std::vector<double>>;
using ExecutionStatusTuple = std::tuple<std::string, std::string>;

//类型转换
template <typename T>
std::string TYPE_TO_JS()
{
    TRANS_TPYE(bool, boolean);
    TRANS_TPYE(std::string, string);
    TRANS_TPYE(const std::string &, string);
    TRANS_TPYE(const std::vector<std::string> &, Array<string>);
    TRANS_TPYE(std::vector<std::string>, Array<string>);
    TRANS_TPYE(double, number);
    TRANS_TPYE(char, number);
    TRANS_TPYE(int, number);
    TRANS_TPYE(float, number);
    TRANS_TPYE(int16_t, number);
    TRANS_TPYE(int32_t, number);
    TRANS_TPYE(uint16_t, number);
    TRANS_TPYE(uint64_t, number);
    TRANS_TPYE(uint32_t, number);
    TRANS_TPYE(const std::vector<uint8_t> &, Array<number>);
    TRANS_TPYE(const std::vector<int32_t> &, Array<number>);
    TRANS_TPYE(std::vector<int>, Array<number>);
    TRANS_TPYE(std::vector<char>, Array<number>);
    TRANS_TPYE(const std::vector<char> &, Array<number>);
    TRANS_TPYE(std::vector<float>, Array<number>);
    TRANS_TPYE(const std::vector<float> &, Array<number>);
    TRANS_TPYE(std::vector<double>, Array<number>);
    TRANS_TPYE(const std::vector<double> &, Array<number>);
    TRANS_TPYE(const std::vector<std::vector<double>> &, Array<Array<number>>);
    TRANS_TPYE(std::vector<std::vector<double>>, Array<Array<number>>);
    TRANS_TPYE(const std::vector<bool> &, Array<boolean>);
    TRANS_TPYE(const arcs::common_interface::Box &, Array<number>);
    TRANS_TPYE(const arcs::common_interface::Cylinder &, Array<number>);
    TRANS_TPYE(const arcs::common_interface::Sphere &, Array<number>);

    // tuple
    TRANS_TPYE(PlanContextTuple, PlanContextTuple);
    TRANS_TPYE(ExecutionStatusTuple, ExecutionStatusTuple);
    TRANS_TPYE(WorkObjectHoldTuple, WorkObjectHoldTuple);
    TRANS_TPYE(FirmwareUpdateProcessTuple, FirmwareUpdateProcessTuple);
    TRANS_TPYE(arcs::common_interface::ResultWithErrno, ResultWithErrnoTuple);
    TRANS_TPYE(arcs::common_interface::ResultWithErrno1, ResultWithErrnoTuple1);
    TRANS_TPYE(arcs::common_interface::Payload, PayLoadTuple);
    TRANS_TPYE(arcs::common_interface::ForceSensorCalibResult,
               ForceSensorCalibResultTuple);
    TRANS_TPYE(arcs::common_interface::DynamicsModel, DynamicsModelTuple);

    // enum
    TRANS_TPYE(arcs::common_interface::AuboErrorCodes, AuboErrorCodes);
    TRANS_TPYE(arcs::common_interface::RuntimeState, RuntimeState);
    TRANS_TPYE(arcs::common_interface::RobotModeType, RobotModeType);
    TRANS_TPYE(arcs::common_interface::SafetyModeType, SafetyModeType);
    TRANS_TPYE(arcs::common_interface::OperationalModeType,
               OperationalModeType);
    TRANS_TPYE(arcs::common_interface::RobotControlModeType,
               RobotControlModeType);
    TRANS_TPYE(std::vector<arcs::common_interface::JointServoModeType>,
               Array<JointServoModeType>);
    TRANS_TPYE(std::vector<arcs::common_interface::JointStateType>,
               Array<JointStateType>);
    TRANS_TPYE(arcs::common_interface::StandardOutputRunState,
               StandardOutputRunState);
    TRANS_TPYE(arcs::common_interface::StandardInputAction,
               StandardInputAction);
    TRANS_TPYE(arcs::common_interface::SafetyInputAction, SafetyInputAction);
    TRANS_TPYE(arcs::common_interface::SafetyOutputRunState,
               SafetyOutputRunState);
    TRANS_TPYE(arcs::common_interface::TaskFrameType, TaskFrameType);
    TRANS_TPYE(arcs::common_interface::TraceLevel, TraceLevel);

    // struct
    TRANS_TPYE(const arcs::common_interface::RobotSafetyParameterRange &, any);
    TRANS_TPYE(const arcs::common_interface::CircleParameters &, any);
    TRANS_TPYE(arcs::common_interface::RobotMsgVector, Array<RobotMsg>);

    // unordered_map
    TRANS_TPYE(DHParam, DHParam);

    // unordered_set
    TRANS_TPYE(const arcs::common_interface::TaskSet &, TaskSet);
};

//萃取参数类型
template <typename T>
using arg1_type_t =
    typename arcs::common_interface::function_traits<T>::template args<0>::type;
template <typename T>
using arg2_type_t =
    typename arcs::common_interface::function_traits<T>::template args<1>::type;
template <typename T>
using arg3_type_t =
    typename arcs::common_interface::function_traits<T>::template args<2>::type;
template <typename T>
using arg4_type_t =
    typename arcs::common_interface::function_traits<T>::template args<3>::type;
template <typename T>
using arg5_type_t =
    typename arcs::common_interface::function_traits<T>::template args<4>::type;
template <typename T>
using arg6_type_t =
    typename arcs::common_interface::function_traits<T>::template args<5>::type;
template <typename T>
using arg7_type_t =
    typename arcs::common_interface::function_traits<T>::template args<6>::type;
template <typename T>
using arg8_type_t =
    typename arcs::common_interface::function_traits<T>::template args<7>::type;
template <typename T>
using arg9_type_t =
    typename arcs::common_interface::function_traits<T>::template args<8>::type;
template <typename T>
using arg10_type_t =
    typename arcs::common_interface::function_traits<T>::template args<9>::type;

//萃取返回值类型
template <typename T>
using return_type_t =
    typename arcs::common_interface::function_traits<T>::return_type;

std::vector<std::string> args_t;
#define GET_ARGS_TYPE1(func) \
    args_t.clear();          \
    args_t.push_back(TYPE_TO_JS<arg1_type_t<decltype(func)>>());

#define GET_ARGS_TYPE2(func)                                     \
    args_t.clear();                                              \
    args_t.push_back(TYPE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg2_type_t<decltype(func)>>());
#define GET_ARGS_TYPE3(func)                                     \
    args_t.clear();                                              \
    args_t.push_back(TYPE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg3_type_t<decltype(func)>>());
#define GET_ARGS_TYPE4(func)                                     \
    args_t.clear();                                              \
    args_t.push_back(TYPE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg4_type_t<decltype(func)>>());
#define GET_ARGS_TYPE5(func)                                     \
    args_t.clear();                                              \
    args_t.push_back(TYPE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg4_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg5_type_t<decltype(func)>>());
#define GET_ARGS_TYPE6(func)                                     \
    args_t.clear();                                              \
    args_t.push_back(TYPE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg4_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg5_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg6_type_t<decltype(func)>>());
#define GET_ARGS_TYPE10(func)                                    \
    args_t.clear();                                              \
    args_t.push_back(TYPE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg4_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg5_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg6_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg7_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg8_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg9_type_t<decltype(func)>>()); \
    args_t.push_back(TYPE_TO_JS<arg10_type_t<decltype(func)>>());

#define JS_FUNC0(ClassType, method, ...)                        \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype( \
        &arcs::common_interface::ClassType::method)>>();        \
    js.defineFunction(#method, method##_t);

#define JS_FUNC1(ClassType, method, arg1)                       \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype( \
        &arcs::common_interface::ClassType::method)>>();        \
    GET_ARGS_TYPE1(&arcs::common_interface::ClassType::method); \
    js.defineFunction(#method, method##_t, #arg1, args_t[0]);

#define JS_FUNC2(ClassType, method, arg1, arg2)                 \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype( \
        &arcs::common_interface::ClassType::method)>>();        \
    GET_ARGS_TYPE2(&arcs::common_interface::ClassType::method); \
    js.defineFunction(#method, method##_t, #arg1, args_t[0], #arg2, args_t[1]);

#define JS_FUNC3(ClassType, method, arg1, arg2, arg3)                          \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype(                \
        &arcs::common_interface::ClassType::method)>>();                       \
    GET_ARGS_TYPE3(&arcs::common_interface::ClassType::method);                \
    js.defineFunction(#method, method##_t, #arg1, args_t[0], #arg2, args_t[1], \
                      #arg3, args_t[2]);

#define JS_FUNC4(ClassType, method, arg1, arg2, arg3, arg4)                    \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype(                \
        &arcs::common_interface::ClassType::method)>>();                       \
    GET_ARGS_TYPE4(&arcs::common_interface::ClassType::method);                \
    js.defineFunction(#method, method##_t, #arg1, args_t[0], #arg2, args_t[1], \
                      #arg3, args_t[2], #arg4, args_t[3]);

#define JS_FUNC5(ClassType, method, arg1, arg2, arg3, arg4, arg5)              \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype(                \
        &arcs::common_interface::ClassType::method)>>();                       \
    GET_ARGS_TYPE5(&arcs::common_interface::ClassType::method);                \
    js.defineFunction(#method, method##_t, #arg1, args_t[0], #arg2, args_t[1], \
                      #arg3, args_t[2], #arg4, args_t[3], #arg5, args_t[4]);
#define JS_FUNC6(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6)        \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype(                \
        &arcs::common_interface::ClassType::method)>>();                       \
    GET_ARGS_TYPE6(&arcs::common_interface::ClassType::method);                \
    js.defineFunction(#method, method##_t, #arg1, args_t[0], #arg2, args_t[1], \
                      #arg3, args_t[2], #arg4, args_t[3], #arg5, args_t[4],    \
                      #arg6, args_t[5]);
#define JS_FUNC10(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6, arg7, \
                  arg8, arg9, arg10)                                           \
    std::string method##_t = TYPE_TO_JS<return_type_t<decltype(                \
        &arcs::common_interface::ClassType::method)>>();                       \
    GET_ARGS_TYPE10(&arcs::common_interface::ClassType::method);               \
    js.defineFunction(#method, method##_t, #arg1, args_t[0], #arg2, args_t[1], \
                      #arg3, args_t[2], #arg4, args_t[3], #arg5, args_t[4],    \
                      #arg6, args_t[5], #arg7, args_t[6], #arg8, args_t[7],    \
                      #arg9, args_t[8], #arg10, args_t[9]);

#endif
