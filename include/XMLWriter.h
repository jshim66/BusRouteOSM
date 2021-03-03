#ifndef XMLWRITER_H
#define XMLWRITER_H

#include "XMLEntity.h"
#include <stack>
#include <istream>
#include <list>

class CXMLWriter{
    private:
        std::ostream &DOutput;
        std::list<std::string> flushbuffer;
    public:
        CXMLWriter(std::ostream &os);
        ~CXMLWriter();

        bool Flush();
        bool WriteEntity(const SXMLEntity &entity);
};

#endif