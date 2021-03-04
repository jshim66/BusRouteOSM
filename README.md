# BusRouteOSM

BusRouteOSM is a program that takes in an OSM(OpenStreetMap) file and finds the shortest distance between bus stops.

# How it works

Through the MapRouter.cpp file, it loads in an OSM file and two CSV(comma-separated) files, named stops and routes

It converts the OSM file to an XML(Extensible Markup Language) file and reads the data using XMLReader.cpp that I have created

It takes in the data through the LoadMapsandRoutes function and loads the entirety of the bus route system while calculating the distance using the Haversine distance method

After creating the general layout of the map, the function reads in the stop and route file and generates the routes for each bus

The overall program can find the shortest path from each stop using Dijkstra's algorithm

# Files

All .cpp files and their respective .h files other than the testing files were created by myself

# Testing

Unfortunately, this program was tested using my professor's test system which consisted of other testing files that are required for the project and it is not currently available.


