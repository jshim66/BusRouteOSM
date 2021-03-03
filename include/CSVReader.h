#ifndef CSVREADER_H 	  			 	 
#define CSVREADER_H

#include <istream>
#include <string>
#include <vector>
#include <list>
#include "csv.h"



class CCSVReader{
    protected:
        csv_parser DParser;
        std::istream &DInput;
        std::vector <std::string > DCurrentRow;
        std::list < std::vector < std::string >> DBufferedRows;

        static void EndOfColumn(void *str, size_t len, void *reader);
        static void EndOfRow(int ch, void *reader);

        
    public:
        CCSVReader(std::istream &in);
        ~CCSVReader();
        
        bool End() const;
        bool ReadRow(std::vector< std::string > &row);
};

#endif
