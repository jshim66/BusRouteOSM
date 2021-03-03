#include "../include/CSVWriter.h"
#include <iostream>
#include <string>
//#include <string.h>
#include "../include/StringUtils.h"

CCSVWriter::CCSVWriter(std::ostream &ou) : DOutput(ou) {

}

CCSVWriter::~CCSVWriter() {
}

bool CCSVWriter::WriteRow(const std::vector<std::string> &row) {

    std::string result;
    std::vector<std::string> temp_vector;

    for(auto &element: row) {
        std::string temp_element = StringUtils::Replace(element, "\"", "\"\"");
        std::string temp = "\"";
        if (element != "") {
            temp += StringUtils::Strip(temp_element); // remove unnecessary whitespace
        }
        temp += "\"";
        temp_vector.push_back(temp);
    }

    result = StringUtils::Join(",", temp_vector);
    result += "\n";

    DOutput<<result;

    return true;

}