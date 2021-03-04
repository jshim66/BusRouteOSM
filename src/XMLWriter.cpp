#include "../include/XMLWriter.h"
#include <iostream>
#include <string>
#include "../include/StringUtils.h"
//https://www.w3schools.com/xml/xml_syntax.asp

CXMLWriter::CXMLWriter(std::ostream &os) : DOutput(os){

}

CXMLWriter::~CXMLWriter(){

}
std::string otherwriteentity(SXMLEntity &entity){
    //https://www.w3schools.com/xml/xml_syntax.asp
    //link explained to us the necessary syntax of XML
    std::string writtenEntity;
    if (entity.DType == SXMLEntity::EType::StartElement){
        writtenEntity += "<" + entity.DNameData;
        for(auto &Attribute : entity.DAttributes){
            Attribute.second = StringUtils::Replace(Attribute.second, "<", "&lt;");
            Attribute.second = StringUtils::Replace(Attribute.second, ">", "gt;");
            Attribute.second = StringUtils::Replace(Attribute.second, "&", "&amp;");
            Attribute.second = StringUtils::Replace(Attribute.second, "\'", "&apos;");
            Attribute.second = StringUtils::Replace(Attribute.second, "\"\"", "&quot;");

            writtenEntity += " " + Attribute.first + "=" + "\"" + Attribute.second + "\"";
        }
        writtenEntity += ">";
        return writtenEntity;
    }
    if (entity.DType == SXMLEntity::EType::EndElement){
        writtenEntity += "</" + entity.DNameData + ">";
        return writtenEntity;

    }
    if (entity.DType == SXMLEntity::EType::CharData){
        writtenEntity = entity.DNameData;
        for(auto &Attribute : entity.DAttributes) {
            Attribute.second = StringUtils::Replace(Attribute.second, "<", "&lt;");
            Attribute.second = StringUtils::Replace(Attribute.second, ">", "gt;");
            Attribute.second = StringUtils::Replace(Attribute.second, "&", "&amp;");
            Attribute.second = StringUtils::Replace(Attribute.second, "\'", "&apos;");
            Attribute.second = StringUtils::Replace(Attribute.second, "\"\"", "&quot;");
        }
        return writtenEntity;

    }
    if (entity.DType == SXMLEntity::EType::CompleteElement){
        writtenEntity += "<" + entity.DNameData;
        for(auto &Attribute : entity.DAttributes){
            Attribute.second = StringUtils::Replace(Attribute.second, "<", "&lt;");
            Attribute.second = StringUtils::Replace(Attribute.second, ">", "gt;");
            Attribute.second = StringUtils::Replace(Attribute.second, "&", "&amp;");
            Attribute.second = StringUtils::Replace(Attribute.second, "\'", "&apos;");
            Attribute.second = StringUtils::Replace(Attribute.second, "\"\"", "&quot;");

            writtenEntity += " " + Attribute.first + "=" + "\"" + Attribute.second + "\"";
        }
        writtenEntity += "/>";
        return writtenEntity;
    }
}
bool CXMLWriter::Flush(){
    if(!flushbuffer.empty()){
        DOutput<< "\n</" +  flushbuffer.front()+ ">";
        flushbuffer.pop_front();
        return true;
    }
    else{
        return false;
    }

}

bool CXMLWriter::WriteEntity(const SXMLEntity &entity){
    std::string str;
    SXMLEntity tentity = entity;
    if(tentity.DType == SXMLEntity::EType()){
        if(tentity.DType == SXMLEntity::EType::StartElement){
            flushbuffer.push_back(entity.DNameData);
        }
        if(tentity.DType == SXMLEntity::EType::EndElement){
            if(!flushbuffer.empty()){
                flushbuffer.pop_back();
            }
        }
        str = otherwriteentity(tentity);
    }
    DOutput << str;
    return true;
}
