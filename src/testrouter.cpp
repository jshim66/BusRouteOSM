#include <gtest/gtest.h>
#include "../include/MapRouter.h"
#include <sstream>
#include "MapRouter.cpp"
#include "StringUtils.cpp"
#include "CSVReader.cpp"
#include "XMLWriter.cpp"
#include "XMLReader.cpp"
#include "CSVWriter.cpp"
#include "../include/csv.h"
#include "../include/XMLReader.h"
#include <iostream>

const char OSMFileData [] = "<?xml version='1.0' encoding='UTF-8'?>\n"
                            "<osm version=\"0.6\" generator=\"osmconvert 0.8.5\">\n"
                            "    <node id=\"1\" lat=\"0\" lon=\"0\"/>\n"
                            "    <node id=\"2\" lat=\"0\" lon=\"1\"/>\n"
                            "    <node id=\"3\" lat=\"0\" lon=\"2\"/>\n"
                            "    <node id=\"4\" lat=\"1\" lon=\"2\"/>\n"
                            "    <node id=\"5\" lat=\"1\" lon=\"1\"/>\n"
                            "    <node id=\"6\" lat=\"1\" lon=\"0\"/>\n"
                            "    <way id=\"11\">\n"
                            "        <nd ref=\"1\"/>\n"
                            "        <nd ref=\"2\"/>\n"
                            "        <nd ref=\"3\"/>\n"
                            "        <nd ref=\"4\"/>\n"
                            "        <nd ref=\"5\"/>\n"
                            "        <nd ref=\"6\"/>\n"
                            "        <tag k=\"oneway\" v=\"yes\"/>\n"
                            "        <tag k=\"highway\" v=\"residential\"/>\n"
                            "        <tag k=\"name\" v=\"Main Street\"/>\n"
                            "    </way>\n"
                            "    <way id=\"12\">\n"
                            "        <nd ref=\"6\"/>\n"
                            "        <nd ref=\"5\"/>\n"
                            "        <nd ref=\"2\"/>\n"
                            "        <nd ref=\"1\"/>\n"
                            "        <tag k=\"oneway\" v=\"yes\"/>\n"
                            "        <tag k=\"highway\" v=\"residential\"/>\n"
                            "        <tag k=\"name\" v=\"Back Street\"/>\n"
                            "    </way>\n"
                            "    <way id=\"13\">\n"
                            "        <nd ref=\"5\"/>\n"
                            "        <nd ref=\"3\"/>\n"
                            "        <tag k=\"oneway\" v=\"yes\"/>\n"
                            "        <tag k=\"highway\" v=\"residential\"/>\n"
                            "        <tag k=\"name\" v=\"Diagonal Street\"/>\n"
                            "    </way>\n"
                            "</osm>\n";

const char StopsCSVFileData[] = "stop_id,node_id\n"
                                "20,2\n"
                                "21,3\n"
                                "22,4\n"
                                "23,5";
                                
const char RoutesCSVFileData[] =    "route,stop_id\n"
                                    "A,20\n"
                                    "A,21\n"
                                    "A,23";
TEST(MapRouter,BasicTest){
    std::stringstream InOSM(OSMFileData);
    std::stringstream InStops(StopsCSVFileData);
    std::stringstream InRoutes(RoutesCSVFileData);
    CMapRouter MapRouter;
        
    EXPECT_TRUE(MapRouter.LoadMapAndRoutes(InOSM, InStops, InRoutes));
    EXPECT_EQ(MapRouter.NodeCount(), 6);
    EXPECT_EQ(MapRouter.GetSortedNodeIDByIndex(0), 1);
    EXPECT_EQ(MapRouter.GetSortedNodeIDByIndex(1), 2);
    EXPECT_EQ(MapRouter.GetSortedNodeIDByIndex(2), 3);
    EXPECT_EQ(MapRouter.GetSortedNodeIDByIndex(3), 4);
    EXPECT_EQ(MapRouter.GetSortedNodeIDByIndex(4), 5);
    EXPECT_EQ(MapRouter.GetSortedNodeIDByIndex(5), 6);
    auto Position = MapRouter.GetSortedNodeLocationByIndex(0);
    EXPECT_EQ(std::get<0>(Position), 0.0);
    EXPECT_EQ(std::get<1>(Position), 0.0);
    Position = MapRouter.GetSortedNodeLocationByIndex(1);
    EXPECT_EQ(std::get<0>(Position), 0.0);
    EXPECT_EQ(std::get<1>(Position), 1.0);
    Position = MapRouter.GetSortedNodeLocationByIndex(2);
    EXPECT_EQ(std::get<0>(Position), 0.0);
    EXPECT_EQ(std::get<1>(Position), 2.0);
    Position = MapRouter.GetSortedNodeLocationByIndex(3);
    EXPECT_EQ(std::get<0>(Position), 1.0);
    EXPECT_EQ(std::get<1>(Position), 2.0);
    Position = MapRouter.GetSortedNodeLocationByIndex(4);
    EXPECT_EQ(std::get<0>(Position), 1.0);
    EXPECT_EQ(std::get<1>(Position), 1.0);
    Position = MapRouter.GetSortedNodeLocationByIndex(5);
    EXPECT_EQ(std::get<0>(Position), 1.0);
    EXPECT_EQ(std::get<1>(Position), 0.0);
    Position = MapRouter.GetSortedNodeLocationByIndex(6);
    EXPECT_EQ(std::get<0>(Position), 180.0);
    EXPECT_EQ(std::get<1>(Position), 360.0);
    
    EXPECT_EQ(MapRouter.RouteCount(), 1);
    EXPECT_EQ(MapRouter.GetSortedRouteNameByIndex(0), "A");
    EXPECT_EQ(MapRouter.GetNodeIDByStopID(20), 2);
    EXPECT_EQ(MapRouter.GetNodeIDByStopID(21), 3);
    EXPECT_EQ(MapRouter.GetNodeIDByStopID(22), 4);
    EXPECT_EQ(MapRouter.GetNodeIDByStopID(23), 5);
    std::vector< CMapRouter::TStopID > Stops;
    
    EXPECT_TRUE(MapRouter.GetRouteStopsByRouteName("A",Stops));
    EXPECT_EQ(Stops.size(), 3);
    EXPECT_EQ(Stops[0], 20);
    EXPECT_EQ(Stops[1], 21);
    EXPECT_EQ(Stops[2], 23);
}

TEST(MapRouter,ShortestPathTest){
    std::stringstream InOSM(OSMFileData);
    std::stringstream InStops(StopsCSVFileData);
    std::stringstream InRoutes(RoutesCSVFileData);
    CMapRouter MapRouter;
    std::vector< CMapRouter::TNodeID > Path;
    
    EXPECT_TRUE(MapRouter.LoadMapAndRoutes(InOSM, InStops, InRoutes));
    EXPECT_EQ(MapRouter.FindShortestPath(4, 3, Path), 69.1129439838730519);
    EXPECT_EQ(Path.size(), 2);
    if(2 == Path.size()){
        EXPECT_EQ(Path[0], 4);
        EXPECT_EQ(Path[1], 3);
    }
}
