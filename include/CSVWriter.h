#ifndef CSVWRITER_H 	  			 	 
#define CSVWRITER_H

#include <ostream>
#include <string>
#include <vector>
#include "csv.h"

class CCSVWriter{
    public:
        std::ostream &DOutput;
        CCSVWriter(std::ostream &ou);
        ~CCSVWriter();

        bool WriteRow(const std::vector< std::string > &row);
};

#endif
