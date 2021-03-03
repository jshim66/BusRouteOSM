#include <iostream>
#include "../include/XMLReader.h"
//www.xml.com/pub/a/1999/09/expat/reference.html#setuserdata - helping us with handler functions

CXMLReader::CXMLReader(std::istream &is) : DInput(is){
    DParser = XML_ParserCreate(nullptr);

    XML_SetUserData(DParser, this);
    XML_SetElementHandler(DParser, ReadStartElement, ReadEndElement);
    XML_SetCharacterDataHandler(DParser, ReadCharData);

}
CXMLReader::~CXMLReader(){
    XML_ParserFree(DParser);
}

void CXMLReader::ReadStartElement(void *data, const char *element, const char *attribute[]){
    // Each attribute seen in a start (or empty) tag occupies 2 consecutive places in this vector
    auto *pointer = static_cast<CXMLReader *>(data);
    SXMLEntity tElement;
    tElement.DType = SXMLEntity::EType::StartElement;
    tElement.DNameData = std::string(element);
    for (int i = 0; attribute[i]; i += 2) {
        tElement.SetAttribute(std::string(attribute[i]), std::string(attribute[i + 1]));
    }
    pointer->DBufferedXml.push_back(tElement);
}

void CXMLReader::ReadEndElement(void *data, const char *element){
    auto *pointer = static_cast<CXMLReader *>(data);
    SXMLEntity tElement;
    tElement.DType = SXMLEntity::EType::EndElement;
    tElement.DNameData = std::string(element);
    pointer->DBufferedXml.push_back(tElement);

}
void CXMLReader::ReadCharData(void *data, const char *content, int length){
    // Set a text handler. The string your handler receives is NOT zero terminated.
    // You have to use the length argument to deal with the end of the string.
    // A single block of contiguous text free of markup may still result in a sequence of calls to this handler.
    // In other words, if you're searching for a pattern in the text, it may be split across calls to this handler.
    auto *pointer = static_cast<CXMLReader *>(data);
    SXMLEntity tElement;
    tElement.DType = SXMLEntity::EType::CharData;
    tElement.DNameData = std::string(content, length);
    pointer->DBufferedXml.push_back(tElement);
}

bool CXMLReader::End(){
    if (not DInput.eof()){
        DInput.peek();
    }
    return DInput.eof() and DBufferedXml.empty();
}

bool CXMLReader::ReadEntity(SXMLEntity &entity, bool skipcdata){
    char Buffer[1024] = "";
    DInput.read(Buffer, sizeof(Buffer));
    XML_Parse(DParser, Buffer, DInput.gcount(), DInput.eof());

    if(!DBufferedXml.empty()){
        if (skipcdata) {
            while (DBufferedXml.front().DType == SXMLEntity::EType::CharData){
                DBufferedXml.pop_front();
            }
        }
    }
    if(DBufferedXml.empty()){
        return false;
    }
    entity = DBufferedXml.front();
    DBufferedXml.pop_front();
    return true;





}