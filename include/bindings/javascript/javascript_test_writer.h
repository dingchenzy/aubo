#ifndef JAVASCRIPT_MOCK_WRITER_H
#define JAVASCRIPT_MOCK_WRITER_H
#include <sstream>
#include <iomanip>
#include "bindings/function_traits.h"
#include "bindings/javascript/javascript_writer.h"
class JavascriptTestWriter
{
public:
    JavascriptTestWriter(std::ostream &os) : os_(os) {}
#define INDENT std::setw(indent_ * 2) << ""
    JavascriptTestWriter &defineImport(const std::string &arg1,
                                       const std::string &arg2)
    {
        os_ << "import { " << arg1 << " } from " << '"' << arg2 << '"'
            << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineClass(const std::string &classname)
    {
        os_ << INDENT << "\nexport class " << classname << " {" << std::endl;
        indent_++;
        return *this;
    }

    JavascriptTestWriter &endClass()
    {
        if (indent_ > 0) {
            indent_--;
        }
        os_ << INDENT << "}" << std::endl;
        return *this;
    }

    JavascriptTestWriter &constructorForMockServer()
    {
        os_ << INDENT << "private _server: Server;" << std::endl;

        this->writeLine("constructor() {");
        this->writeLine("  this._server = new Server({");
        this->writeLine("    port: 8080,");
        this->writeLine("    host: 'localhost',");
        this->writeLine("  });");
        this->writeLine("}\n");
        return *this;
    }

    JavascriptTestWriter &definefunctionForStop()
    {
        this->writeLine("public stop() {");
        this->writeLine("    this._server.close();");
        this->writeLine("  }");
        return *this;
    }

    JavascriptTestWriter &defineRegisterForRobot(const std::string &classname,
                                                 const std::string &method,
                                                 const std::string &ret)
    {
        os_ << INDENT
            << "  this._server.register('rob1." + classname + "." + method +
                   "', function (params) {"
            << std::endl;
        os_ << INDENT << "    console.log(" << std::endl;
        os_ << INDENT
            << R"(      "---> {'method':'rob1.)" + classname + "." + method +
                   R"(','params':%O}",params)"
            << std::endl;
        os_ << INDENT << "    );" << std::endl;
        os_ << INDENT << "    return " << ret << ";" << std::endl;
        os_ << INDENT << "  });\n" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineRegister(const std::string &classname,
                                         const std::string &method,
                                         const std::string &ret)
    {
        os_ << INDENT
            << "  this._server.register('" + classname + "." + method +
                   "', function (params) {"
            << std::endl;
        os_ << INDENT << "    console.log(" << std::endl;
        os_ << INDENT
            << R"(      "---> {'method':')" + classname + "." + method +
                   R"(','params':%O}",params)"
            << std::endl;
        os_ << INDENT << "    );" << std::endl;
        os_ << INDENT << "    return " << ret << ";" << std::endl;
        os_ << INDENT << "  });\n" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineDescribeForRobot(const std::string &classname)
    {
        os_ << "describe('" << classname << "', async () => {" << std::endl;
        os_ << INDENT << "let rpc_cli = new Client('ws://localhost:8080');"
            << std::endl;
        os_ << INDENT << "let clazz = new " << classname << "(rpc_cli, 'rob1');"
            << std::endl;
        os_ << INDENT
            << "new Promise(() => { rpc_cli.on('open', () => "
               "{});"
            << std::endl;
        os_ << INDENT << "});" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineDescribe(const std::string &classname)
    {
        os_ << "describe('" << classname << "', async () => {" << std::endl;
        os_ << INDENT << "let rpc_cli = new Client('ws://localhost:8080');"
            << std::endl;
        os_ << INDENT << "let clazz = new " << classname << "(rpc_cli);"
            << std::endl;
        os_ << INDENT
            << "new Promise(() => { rpc_cli.on('open', () => "
               "{});"
            << std::endl;
        os_ << INDENT << "});" << std::endl;
        return *this;
    }
    JavascriptTestWriter &defineForIt(const std::string &classname,
                                      const std::string &method,
                                      const std::string &ret)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "();"
            << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(" << ret << ");"
            << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineForIt(const std::string &classname,
                                      const std::string &method,
                                      const std::string &ret,
                                      const std::string &arg1)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "("
            << arg1 << ");" << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(" << ret << ");"
            << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineForIt(const std::string &classname,
                                      const std::string &method,
                                      const std::string &ret,
                                      const std::string &arg1,
                                      const std::string &arg2)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "("
            << arg1 << ", " << arg2 << ");" << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(" << ret << ");"
            << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineForIt(const std::string &classname,
                                      const std::string &method,
                                      const std::string &ret,
                                      const std::string &arg1,
                                      const std::string &arg2,
                                      const std::string &arg3)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "("
            << arg1 << ", " << arg2 << ", " << arg3 << ");" << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(" << ret << ");"
            << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineForIt(const std::string &classname,
                                      const std::string &method,
                                      const std::string &ret,
                                      const std::string &arg1,
                                      const std::string &arg2,
                                      const std::string &arg3,
                                      const std::string &arg4)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "("
            << arg1 << ", " << arg2 << ", " << arg3 << ", " << arg4 << ");"
            << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(" << ret << ");"
            << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }
    JavascriptTestWriter &defineForIt(
        const std::string &classname, const std::string &method,
        const std::string &ret, const std::string &arg1,
        const std::string &arg2, const std::string &arg3,
        const std::string &arg4, const std::string &arg5)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "("
            << arg1 << ", " << arg2 << ", " << arg3 << ", " << arg4 << ", "
            << arg5 << ");" << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(" << ret << ");"
            << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineForIt(
        const std::string &classname, const std::string &method,
        const std::string &ret, const std::string &arg1,
        const std::string &arg2, const std::string &arg3,
        const std::string &arg4, const std::string &arg5,
        const std::string &arg6)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "("
            << arg1 << ", " << arg2 << ", " << arg3 << ", " << arg4 << ", "
            << arg5 << ", " << arg6 << ");" << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(" << ret << ");"
            << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }

    JavascriptTestWriter &defineForIt(
        const std::string &classname, const std::string &method,
        const std::string &ret, const std::string &arg1,
        const std::string &arg2, const std::string &arg3,
        const std::string &arg4, const std::string &arg5,
        const std::string &arg6, const std::string &arg7,
        const std::string &arg8, const std::string &arg9,
        const std::string &arg10)
    {
        os_ << INDENT << "it('" << classname << "." << method
            << "', async function () {" << std::endl;
        os_ << INDENT << "    let result = await clazz." << method << "("
            << arg1 << ", " << arg2 << ", " << arg3 << ", " << arg4 << ", "
            << arg5 << ", " << arg6 << ", " << arg7 << ", " << arg8 << ", "
            << arg9 << ", " << arg10 << ");" << std::endl;
        os_ << INDENT << "    console.log(\"<--- {'result':%O}\", result);"
            << std::endl;
        os_ << INDENT << "    expect(result).to.deep.equal(0);" << std::endl;
        os_ << INDENT << "  });" << std::endl;
        return *this;
    }

    JavascriptTestWriter &writeLine(const std::string &content)
    {
        os_ << INDENT << content << std::endl;
        return *this;
    }
    std::ostream &os() { return os_; }

private:
    std::ostream &os_;
    int indent_ = 0;
};
#endif

#define SET_VALUE(CPP_TYPE, JS_VALUE)            \
    if constexpr (std::is_same<T, CPP_TYPE>()) { \
        return JS_VALUE;                         \
    }

//类型转换
template <typename T>
std::string VALUE_TO_JS()
{
    SET_VALUE(bool, "true");
    SET_VALUE(std::string, "'str'");
    SET_VALUE(const std::string &, "'str'");
    SET_VALUE(const std::vector<std::string> &, "new Array(6).fill('str')");
    SET_VALUE(std::vector<std::string>, "new Array(6).fill('str')");
    SET_VALUE(double, "0.0");
    SET_VALUE(char, "0");
    SET_VALUE(int, "0");
    SET_VALUE(float, "0.0");
    SET_VALUE(int16_t, "0");
    SET_VALUE(int32_t, "0");
    SET_VALUE(uint16_t, "0");
    SET_VALUE(uint64_t, "0");
    SET_VALUE(uint32_t, "0");
    SET_VALUE(const std::vector<uint8_t> &, "new Array(6).fill(0)");
    SET_VALUE(const std::vector<int32_t> &, "new Array(6).fill(0)");
    SET_VALUE(std::vector<int>, "new Array(6).fill(0)");
    SET_VALUE(std::vector<char>, "new Array(6).fill(0)");
    SET_VALUE(const std::vector<char> &, "new Array(6).fill(0)");
    SET_VALUE(std::vector<float>, "new Array(6).fill(0.0)");
    SET_VALUE(const std::vector<float> &, "new Array(6).fill(0.0)");
    SET_VALUE(std::vector<double>, "new Array(6).fill(0.0)");
    SET_VALUE(const std::vector<double> &, "new Array(6).fill(0.0)");
    SET_VALUE(const std::vector<std::vector<double>> &,
              "[new Array(6).fill(0.0)]");
    SET_VALUE(std::vector<std::vector<double>>, "[new Array(6).fill(0.0)]");
    SET_VALUE(const std::vector<bool> &, "new Array(6).fill(true)");
    SET_VALUE(const arcs::common_interface::Box &, "new Array(6).fill(0.0)");
    SET_VALUE(const arcs::common_interface::Cylinder &,
              "new Array(5).fill(0.0)");
    SET_VALUE(const arcs::common_interface::Sphere &, "new Array(3).fill(0.0)");

    // tuple
    SET_VALUE(PlanContextTuple, "[0, 0, 'str']");
    SET_VALUE(ExecutionStatusTuple, "['str', 'str']");
    SET_VALUE(WorkObjectHoldTuple, "[ 'str', new Array(6).fill(0.0)]");
    SET_VALUE(FirmwareUpdateProcessTuple, "['str', 0.0]");
    SET_VALUE(arcs::common_interface::ResultWithErrno,
              "[new Array(6).fill(0.0), 0]");
    SET_VALUE(arcs::common_interface::ResultWithErrno1,
              "[new Array(6).fill(0.0), 0]");
    SET_VALUE(arcs::common_interface::Payload,
              "[0, new Array(6).fill(0.0), new Array(6).fill(0.0), new "
              "Array(6).fill(0.0)]");
    SET_VALUE(arcs::common_interface::ForceSensorCalibResult,
              "[new Array(6).fill(0.0), new Array(6).fill(0.0), 0, new "
              "Array(6).fill(0.0)]");
    SET_VALUE(arcs::common_interface::DynamicsModel,
              "new Array(3).fill(new Array(6).fill(0.0))");

    // enum
    SET_VALUE(arcs::common_interface::AuboErrorCodes, "AuboErrorCodes.AUBO_OK");
    SET_VALUE(arcs::common_interface::RuntimeState, "RuntimeState.Stopped");
    SET_VALUE(arcs::common_interface::RobotModeType,
              "RobotModeType.NoController");
    SET_VALUE(arcs::common_interface::SafetyModeType,
              "SafetyModeType.Undefined");
    SET_VALUE(arcs::common_interface::OperationalModeType,
              "OperationalModeType.Automatic");
    SET_VALUE(arcs::common_interface::RobotControlModeType,
              "RobotControlModeType.None");
    SET_VALUE(std::vector<arcs::common_interface::JointServoModeType>,
              "new Array(6).fill(JointServoModeType.None)");
    SET_VALUE(std::vector<arcs::common_interface::JointStateType>,
              "new Array(6).fill(JointStateType.Idle)");
    SET_VALUE(arcs::common_interface::StandardOutputRunState,
              "StandardOutputRunState.None");
    SET_VALUE(arcs::common_interface::StandardInputAction,
              "StandardInputAction.Default");
    SET_VALUE(arcs::common_interface::SafetyInputAction,
              "SafetyInputAction.Unassigned");
    SET_VALUE(arcs::common_interface::SafetyOutputRunState,
              "SafetyOutputRunState.Unassigned");
    SET_VALUE(arcs::common_interface::TaskFrameType, "TaskFrameType.NONE");
    SET_VALUE(arcs::common_interface::TraceLevel, "TraceLevel.INFO");

    // struct
    SET_VALUE(const arcs::common_interface::RobotSafetyParameterRange &, "{}");
    SET_VALUE(const arcs::common_interface::CircleParameters &, "{}");
    SET_VALUE(arcs::common_interface::RobotMsgVector,
              "[new Array(6).fill({ timestamp:0, level:TraceLevel.INFO, code: "
              "0, source: 'str', args: [] })]");

    // unordered_map
    SET_VALUE(DHParam, "{'str': new Array(6).fill(0.0)}");

    // unordered_set
    SET_VALUE(const arcs::common_interface::TaskSet &,
              "new Set(['str1', 'str2'])");
};

std::string return_value;
#define JS_MOCK1(ClassType, method, ...)                 \
    return_value = VALUE_TO_JS<return_type_t<decltype(   \
        &arcs::common_interface::ClassType::method)>>(); \
    js_mock.defineRegisterForRobot(#ClassType, #method, return_value);

#define JS_MOCK2(ClassType, method, ...)                 \
    return_value = VALUE_TO_JS<return_type_t<decltype(   \
        &arcs::common_interface::ClassType::method)>>(); \
    js_mock.defineRegister(#ClassType, #method, return_value);

std::vector<std::string> args_value;
#define GET_ARGS_VALUE1(func) \
    args_value.clear();       \
    args_value.push_back(VALUE_TO_JS<arg1_type_t<decltype(func)>>());

#define GET_ARGS_VALUE2(func)                                         \
    args_value.clear();                                               \
    args_value.push_back(VALUE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg2_type_t<decltype(func)>>());
#define GET_ARGS_VALUE3(func)                                         \
    args_value.clear();                                               \
    args_value.push_back(VALUE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg3_type_t<decltype(func)>>());
#define GET_ARGS_VALUE4(func)                                         \
    args_value.clear();                                               \
    args_value.push_back(VALUE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg4_type_t<decltype(func)>>());
#define GET_ARGS_VALUE5(func)                                         \
    args_value.clear();                                               \
    args_value.push_back(VALUE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg4_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg5_type_t<decltype(func)>>());
#define GET_ARGS_VALUE6(func)                                         \
    args_value.clear();                                               \
    args_value.push_back(VALUE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg4_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg5_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg6_type_t<decltype(func)>>());
#define GET_ARGS_VALUE10(func)                                        \
    args_value.clear();                                               \
    args_value.push_back(VALUE_TO_JS<arg1_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg2_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg3_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg4_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg5_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg6_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg7_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg8_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg9_type_t<decltype(func)>>()); \
    args_value.push_back(VALUE_TO_JS<arg10_type_t<decltype(func)>>());

#define JS_TEST0(ClassType, method, ...)                 \
    return_value = VALUE_TO_JS<return_type_t<decltype(   \
        &arcs::common_interface::ClassType::method)>>(); \
    js_test.defineForIt(#ClassType, #method, return_value);

#define JS_TEST1(ClassType, method, arg1)                        \
    return_value = VALUE_TO_JS<return_type_t<decltype(           \
        &arcs::common_interface::ClassType::method)>>();         \
    GET_ARGS_VALUE1(&arcs::common_interface::ClassType::method); \
    js_test.defineForIt(#ClassType, #method, return_value, args_value[0]);

#define JS_TEST2(ClassType, method, arg1, arg2)                           \
    return_value = VALUE_TO_JS<return_type_t<decltype(                    \
        &arcs::common_interface::ClassType::method)>>();                  \
    GET_ARGS_VALUE2(&arcs::common_interface::ClassType::method);          \
    js_test.defineForIt(#ClassType, #method, return_value, args_value[0], \
                        args_value[1]);

#define JS_TEST3(ClassType, method, arg1, arg2, arg3)                     \
    return_value = VALUE_TO_JS<return_type_t<decltype(                    \
        &arcs::common_interface::ClassType::method)>>();                  \
    GET_ARGS_VALUE3(&arcs::common_interface::ClassType::method);          \
    js_test.defineForIt(#ClassType, #method, return_value, args_value[0], \
                        args_value[1], args_value[2]);

#define JS_TEST4(ClassType, method, arg1, arg2, arg3, arg4)               \
    return_value = VALUE_TO_JS<return_type_t<decltype(                    \
        &arcs::common_interface::ClassType::method)>>();                  \
    GET_ARGS_VALUE4(&arcs::common_interface::ClassType::method);          \
    js_test.defineForIt(#ClassType, #method, return_value, args_value[0], \
                        args_value[1], args_value[2], args_value[3]);

#define JS_TEST5(ClassType, method, arg1, arg2, arg3, arg4, arg5)         \
    return_value = VALUE_TO_JS<return_type_t<decltype(                    \
        &arcs::common_interface::ClassType::method)>>();                  \
    GET_ARGS_VALUE5(&arcs::common_interface::ClassType::method);          \
    js_test.defineForIt(#ClassType, #method, return_value, args_value[0], \
                        args_value[1], args_value[2], args_value[3],      \
                        args_value[4]);

#define JS_TEST6(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6)   \
    GET_ARGS_VALUE6(&arcs::common_interface::ClassType::method);          \
    return_value = VALUE_TO_JS<return_type_t<decltype(                    \
        &arcs::common_interface::ClassType::method)>>();                  \
    js_test.defineForIt(#ClassType, #method, return_value, args_value[0], \
                        args_value[1], args_value[2], args_value[3],      \
                        args_value[4], args_value[5]);

#define JS_TEST10(ClassType, method, arg1, arg2, arg3, arg4, arg5, arg6, arg7, \
                  arg8, arg9, arg10)                                           \
    return_value = VALUE_TO_JS<return_type_t<decltype(                         \
        &arcs::common_interface::ClassType::method)>>();                       \
    GET_ARGS_VALUE10(&arcs::common_interface::ClassType::method);              \
    js_test.defineForIt(#ClassType, #method, return_value, args_value[0],      \
                        args_value[1], args_value[2], args_value[3],           \
                        args_value[4], args_value[5], args_value[6],           \
                        args_value[7], args_value[8], args_value[9]);
