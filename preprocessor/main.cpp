#include <fstream>
#include <string>
#if defined(__linux__) || defined(__linux)
#include "bindings/lua/aubo_control_api.h"
#endif
#include "bindings/jsonrpc/jsonrpc_service.hpp"
#include "bindings/impl/pimpl.h"
#include "aubo/error_stack/error_stack.h"
#include "bindings/rtde/rtde_recipe.h"
#include "code_translator.h"

using namespace arcs::error_stack;

namespace arcs::common_interface {
//纯虚析构要加上空函数体，否则MSVC编译器会出现链接问题，部分编译器会默认自动加上
// https://www.cnblogs.com/chio/archive/2007/09/10/888260.html
DEFINE_CTOR(Trace)
DEFINE_CTOR(Math)
DEFINE_CTOR(SystemInfo)
DEFINE_CTOR(RuntimeMachine)
DEFINE_CTOR(RegisterControl)
DEFINE_CTOR(SyncMove)
DEFINE_CTOR(RobotState)
DEFINE_CTOR(MotionControl)
DEFINE_CTOR(AuboApi)
DEFINE_CTOR(RobotInterface)
DEFINE_CTOR(ForceControl)
DEFINE_CTOR(IoControl)
DEFINE_CTOR(RobotConfig)
DEFINE_CTOR(RobotAlgorithm)
DEFINE_CTOR(RobotManage)
DEFINE_CTOR(Socket)
DEFINE_CTOR(Serial)
} // namespace arcs::common_interface

std::ostream &error_codes_dump(std::ostream &os)
{
    os << "<div align='center'>" << std::endl;
    os << "<h1 align='center'>Aubo Sdk Error Code </h1>" << std::endl;
    os << "</div>\n" << std::endl;
    os << "## 缩写说明\n"
       << "* JNT: joint\n"
       << "* PDL: pedstral\n"
       << "* TP: teach pendant\n"
       << "* COMM: communication\n"
       << "* ENC: encoder\n"
       << "* CURR: current\n"
       << "* POS: position\n"
       << "* PKG: package\n"
       << "* PROG: program\n";
    os << "## 错误码" << std::endl;
    os << "| 错误名称 | 错误码 | 说明 | 建议 |" << std::endl;
    os << "| :----:| :----: | :----: | :----: |" << std::endl;
#define _D(n, v, s, r) \
    os << "|" << #n << "|" << v << "|" << s << "|" << r << "|" << std::endl;
    ARCS_ERROR_CODES
#undef _D

    return os;
}

std::string doc_str;
std::string::size_type offindex;
#define STR_RPLACE(str)                          \
    doc_str = std::string(str);                  \
    offindex = doc_str.find("\n", 0);            \
    while (offindex != std::string::npos) {      \
        doc_str.replace(offindex, 2, "<br>");    \
        offindex += 1;                           \
        offindex = doc_str.find("\n", offindex); \
    }

std::ostream &rtde_recipe_dump(std::ostream &os)
{
    os << "<div align='center'>" << std::endl;
    os << "<h1 align='center'>Aubo Sdk RTDE Recipe </h1>" << std::endl;
    os << "</div>\n" << std::endl;

    os << "## 输入菜单" << std::endl;
    os << "| 名称 | 数据类型 | 说明 |" << std::endl;
    os << "| :----:| :----: | :----: |" << std::endl;
#define RRII(name, enum_number, type, document) \
    STR_RPLACE(document);                       \
    os << "|" << #name << "|" << #type << "|" << doc_str << "|" << std::endl;
    RTDE_INPUT_MAP
#undef RRII

    os << "## 输出菜单" << std::endl;
    os << "| 名称 | 数据类型 | 说明 |" << std::endl;
    os << "| :----:| :----: | :----: |" << std::endl;
#define RRII(name, enum_number, type, document) \
    STR_RPLACE(document);                       \
    os << "|" << #name << "|" << #type << "|" << doc_str << "|" << std::endl;
    RTDE_OUTPUT_MAP
#undef RRII

    return os;
}

int main(int argc, char **argv)
{
    std::ofstream ofs("error_code/error_translator.h");
    std::ofstream error_code_file("error_code/error_code.md", std::ios::trunc);
    CodeTranslator code_trans;
    code_trans.writeHeader(ofs);
    ofs << std::flush;
    std::cout
        << "++++++++++++++++++++++++++::ERRO_CODES::++++++++++++++++++++++++++"
        << std::endl;
    error_codes_dump(error_code_file);
    arcs::error_stack::dump(std::cout);
#if defined(__linux__) || defined(__linux)
    std::cout << "\n++++++++++++++++++++++++++::LUA::++++++++++++++++++++++++++"
              << std::endl;
    arcs::aubo_script::LuaAuboApi::generateTestScript(std::cout);
#endif

    std::cout
        << "\n++++++++++++++++++++++++++::JSONRPC::++++++++++++++++++++++++++"
        << std::endl;
    arcs::common_interface::JsonRpcService::generateSnippets(std::cout);

    // RTDE RECIPE
    std::ofstream rtde_file("rtde_recipe/rtde_recipe.md", std::ios::trunc);
    rtde_recipe_dump(rtde_file);

    return 0;
}
