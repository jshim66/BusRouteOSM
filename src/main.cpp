#include <iostream>
#include "../include/MapRouter.h"
#include "../include/CSVReader.h"
#include "../include/XMLReader.h"
#include "../include/StringUtils.h"
#include <vector>
#include <string>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <istream>
#include <strstream>
#include <cmath>


int main(int argc, char *argv[]){
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    using TNodeID = unsigned long;
    using TPathStep = std::pair<std::string, TNodeID>;
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    using TNodeIndex = unsigned long;
    std::cout<<"@ line: "<<__LINE__<<std::endl;

    //argc is the number of arguements passed in
    //argv is the vector of the arguements
    // Your code here

    if(argc >= 2){
        for(int i = 0; i < argc; i++){
            std::cout<<argv[i]<<std::endl;
        }
    }
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    CMapRouter map;
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    std::ifstream myosm;
    myosm.open("data/davis.osm",std::ios::in);
    std::istream& osm = myosm;
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    std::ifstream mystops;
    mystops.open("data/stops.csv",std::ios::in);
    std::istream& stops = mystops;
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    std::ifstream myroutes;
    myroutes.open("data/routes.csv",std::ios::in);
    std::istream& routes = myroutes;
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    map.LoadMapAndRoutes(osm,stops,routes);
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    bool Done = false;
    std::cout<<"@ line: "<<__LINE__<<std::endl;
    while(!Done){
        std::cout<<"@ line: "<<__LINE__<<std::endl;
        std::string InputLine;std::cout<<"> ";
        std::getline(std::cin, InputLine);
        std::vector <TNodeID> pather;
        std::vector< TPathStep > pather1;
        std::string printer ="";

        std::vector <std::string> InputVector = StringUtils::Split(InputLine," ");
        if(InputVector[0] == "exit"){
            Done = true;
        }
        else if(InputVector[0] == "help"){
            std::cout<<"findroute [--data=path | --resutls=path]\n"
                       "------------------------------------------------------------------------\n"
                       "help     Display this help menu\n"
                       "exit     Exit the program\n"
                       "count    Output the number of nodes in the map\n"
                       "node     Syntax \"node [0, count)\"\n"
                       "         Will output node ID and Lat/Lon for node\n"
                       "fastest  Syntax \"fastest start end\"\n"
                       "         Calculates the time for fastest path from start to end\n"
                       "shortest Syntax \"shortest start end\"\n"
                       "         Calculates the distance for the shortest path from start to end\n"
                       "save     Saves the last calculated path to file\n"
                       "print    Prints the steps for the last calculated path\n";
        }
        else if(InputVector[0] == "count"){
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            std::cout<< map.NodeCount()<<std::endl;

        }
        else if(InputVector[0] == "node"){
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            std::cout << "index is "<< InputVector[1] <<std::endl;
            InputVector[1] = StringUtils::Strip(InputVector[1]);
            std::cout << "index is after stripping "<< InputVector[1] <<std::endl;
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            TNodeIndex index4 = std::stoi(InputVector[1]);
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            auto Nodeid1= map.GetSortedNodeIDByIndex(index4);
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            std::pair <double,double> loca = map.GetSortedNodeLocationByIndex(index4);
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            auto lat1 = std::get<0>(loca);
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            auto long1 = std::get<1>(loca);
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            std::cout<<"Node "<<InputVector[1] <<": "<<"id = "<< Nodeid1 <<" is at "<< lat1 <<", "<< long1 <<std::endl;
        }
        else if(InputVector[0] == "shortest"){
            printer = "short";
            InputVector[1] = StringUtils::Strip(InputVector[1]);
            InputVector[2] = StringUtils::Strip(InputVector[2]);
            TNodeIndex index5 = std::stoi(InputVector[1]);
            TNodeIndex index6 = std::stoi(InputVector[2]);
            //auto index = map.NodeTranslation.find(std::stoul(InputVector[1]))->second;
            //auto otherindex = map.NodeTranslation.find(std::stoul(InputVector[2]))->second;
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            auto dist = map.FindShortestPath(index5,index6,pather);
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            if (dist == -1){
                std::cout<<"@ line: "<<__LINE__<<std::endl;
                std::cout <<"Unable to find shortest path from "<<InputVector[1]<<" to "<<InputVector[2]<<std::endl;
            }
            else{
                std::cout<<"@ line: "<<__LINE__<<std::endl;
                std::cout <<"Shortest path is "<<dist<<"mi"<<std::endl;
            }
        }
        else if(InputVector[0] == "fastest"){
            printer = "fast";
            InputVector[1] = StringUtils::Strip(InputVector[1]);
            InputVector[2] = StringUtils::Strip(InputVector[2]);
            TNodeIndex index5 = std::stoi(InputVector[1]);
            TNodeIndex index6 = std::stoi(InputVector[2]);
            //auto index = map.NodeTranslation.find(std::stoul(InputVector[1]))->second;
            //auto otherindex = map.NodeTranslation.find(std::stoul(InputVector[2]))->second;
            std::cout<<"@ line: "<<__LINE__<<std::endl;

            std::cout<<"@ line: "<<__LINE__<<std::endl;
            auto time = map.FindFastestPath(index5,index6,pather1);
            std::cout<<"@ line: "<<__LINE__<<std::endl;
            if (time == -1){
                std::cout<<"@ line: "<<__LINE__<<std::endl;
                std::cout <<"Unable to find shortest path from "<<InputVector[1]<<" to "<<InputVector[2]<<std::endl;
            }
            else{
                std::cout<<"@ line: "<<__LINE__<<std::endl;
                long flooredtime = floor(time);
                if (time-flooredtime ==0){
                    std::cout <<"Fastest path takes "<<time<<" hours"<<std::endl;
                }
                else{
                    double mint = (time-flooredtime)*60;
                    long floormin = floor(mint);
                    if (mint-floormin == 0){
                        if (flooredtime == 0){
                            std::cout <<"Fastest path takes "<<floormin<<" minutes"<<std::endl;
                        }
                        else{
                            std::cout <<"Fastest path takes "<<flooredtime<<" hours "<<floormin<<"minutes"<<std::endl;
                        }

                    }
                    else{
                        double sec = (mint-floormin)*60;
                        long floorsec = floor(sec);
                        if (flooredtime == 0){
                            if (floormin == 0){
                                std::cout <<"Fastest path takes "<< sec <<" seconds"<<std::endl;
                            }
                            else{
                                std::cout <<"Fastest path takes "<<floormin<<" minutes"<<sec<<" seconds"<<std::endl;
                            }

                        }
                        else{
                            if (floormin == 0){
                                std::cout <<"Fastest path takes "<< flooredtime <<" hours"<<sec<<" seconds"<<std::endl;
                            }
                            else{
                                std::cout <<"Fastest path takes "<< flooredtime <<" hours"<<floormin<<" minutes"<<sec<<" seconds"<<std::endl;
                            }

                        }

                    }
                }
            }
        }
        else if(InputVector[0] == "print"){
            if (printer == "") {
                std::cout << "No valid path to print." << std::endl;
            }
            else if (printer =="short") {
                for (int k = 0; k < pather.size();k++){
                    std::cout<< pather[k]<<std::endl;
                }
            }
            else if (printer == "fast"){
                std::vector< std::string > outputer;
                map.GetPathDescription(pather1,outputer);
                for (int k = 0; k < pather.size();k++){
                    std::cout<< outputer[k]<<std::endl;
                }
            }
        }

    }
    return EXIT_SUCCESS;
}



