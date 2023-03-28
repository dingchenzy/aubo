#ifndef CODETRANSLATOR_H
#define CODETRANSLATOR_H

#include <ostream>

class CodeTranslator
{
public:
    CodeTranslator();

    void writeHeader(std::ostream &os);
    void writeSource(std::ostream &os);
};

#endif // CODETRANSLATOR_H
