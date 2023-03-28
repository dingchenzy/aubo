#include "code_translator.h"

#include <aubo/error_stack/error_stack.h>
#undef _PH1_
#undef _PH2_
#undef _PH3_
#undef _PH4_
#define _PH1_ "%1"
#define _PH2_ "%2"
#define _PH3_ "%3"
#define _PH4_ "%4"

using namespace arcs::error_stack;

CodeTranslator::CodeTranslator()
{
}

void CodeTranslator::writeHeader(std::ostream &os)
{
    os << "// 此文件为自动生成，请勿修改" << std::endl;
    os << "// This file is auto-generated. Please don't modified it."
       << std::endl;
    os << "#include <QCoreApplication>" << std::endl;
    os << "#include <QMap>" << std::endl;
    os << "#include <aubo/error_stack/error_stack.h>" << std::endl;
    os << std::endl;
    os << "class ErrorCode" << std::endl;
    os << "{" << std::endl;
    os << "    Q_DECLARE_TR_FUNCTIONS(ErrorCode)" << std::endl;
    os << "public:" << std::endl;
    os << "ErrorCode()" << std::endl;
    os << "{" << std::endl;
    os << "}" << std::endl;
    os << "static void buildTransMap()" << std::endl;
    os << "{" << std::endl;
#define _D(n, v, s, r)                                                        \
    os << "    trMap[arcs::error_stack::" << #n << "] = tr(\"" << s << "\");" \
       << std::endl;
    ARCS_ERROR_CODES
#undef _D
    os << "}" << std::endl;
    os << "static QString mytr(int err_code)" << std::endl;
    os << "{" << std::endl;
    os << "    if (trMap.contains(err_code)) {" << std::endl;
    os << "        return trMap[err_code];" << std::endl;
    os << "    } else {" << std::endl;
    os << "        return arcs::error_stack::errorCode2Str(err_code);"
       << std::endl;
    os << "    }" << std::endl;
    os << "}" << std::endl;
    os << "private:" << std::endl;
    os << "    inline static QMap<int, QString> trMap;" << std::endl;
    os << "};" << std::endl;
}

void CodeTranslator::writeSource(std::ostream &os)
{
}
