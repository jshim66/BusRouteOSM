#ifndef XMLREADER_H
#define XMLREADER_H

#include "XMLEntity.h"
#include <istream>
#include <expat.h>
#include <queue>
#include <vector>
#include <list>
//www.xml.com/pub/a/1999/09/expat/reference.html#setuserdata

class CXMLReader{
    private:
        std::istream &DInput;
        XML_Parser DParser;
        std::list<SXMLEntity> DBufferedXml;

        static void ReadStartElement(void *data, const char *element, const char *attribute[]);
        static void ReadEndElement(void *data, const char *element);
        static void ReadCharData(void *data, const char *content, int length);

    public:
        CXMLReader(std::istream &is);
        ~CXMLReader();

        bool End();
        bool ReadEntity(SXMLEntity &entity, bool skipcdata = false);
};

#endif