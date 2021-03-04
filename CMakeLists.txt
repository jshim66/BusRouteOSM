cmake_minimum_required(VERSION 3.15)
project(BusRoute)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(libcsv-3.0.3)

add_executable(BusRoute
        data/davis.osm
        data/routes.csv
        data/stops.csv
        include/csv.h
        include/CSVReader.h
        include/CSVWriter.h
        include/MapRouter.h
        include/StringUtils.h
        include/XMLEntity.h
        include/XMLReader.h
        include/XMLWriter.h
        libcsv/csv.h
        src/CSVReader.cpp
        src/CSVWriter.cpp
        src/main.cpp
        src/MapRouter.cpp
        src/StringUtils.cpp
        src/testrouter.cpp
        src/XMLReader.cpp
        src/XMLWriter.cpp
        )
