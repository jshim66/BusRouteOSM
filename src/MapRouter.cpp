#include "../include/MapRouter.h"
#include <cmath>
#include "../include/XMLReader.h"
#include "../include/XMLEntity.h"
#include <algorithm>
#include <ostream>
#include "../include/CSVReader.h"
#include <iostream>       // std::cout
#include <stack>          // std::stack
#include <vector>         // std::vector
#include <deque>
#include <queue>
#include <limits>
#include <utility>
#include <cmath>

const CMapRouter::TNodeID CMapRouter::InvalidNodeID = -1;

CMapRouter::CMapRouter(){

}

CMapRouter::~CMapRouter(){

}
double CMapRouter::Dijsktras(TNodeIndex src, TNodeIndex dest,std::vector<TNodeID>&path){
    std::vector<TNodeIndex> heaptemp;
    std::vector<double> timeholder(Nodes.size(), std::numeric_limits<double>::max());
    std::vector<TNodeID> previousholder(Nodes.size());

    timeholder[src] = 0.00;
    previousholder[src] = src;
    heaptemp.push_back(src);
    auto CompareLambda = [&timeholder](TNodeIndex idx1, TNodeIndex idx2) {
        return timeholder[idx1] < timeholder[idx2];
    };
    while (!heaptemp.empty()) {

        std::make_heap(heaptemp.begin(), heaptemp.end(), CompareLambda);
        auto backtemp = heaptemp.back();
        std::pop_heap(heaptemp.begin(), heaptemp.end(), CompareLambda);
        heaptemp.pop_back();
        for (auto edges: Nodes[backtemp].Edges) {
            double number = 0.00;
            double timetemp = timeholder[backtemp] + edges.time;

            if (timetemp < timeholder[edges.OtherNode]) {
                if (timeholder[edges.OtherNode] == std::numeric_limits<double>::max()) {
                    heaptemp.push_back(edges.OtherNode);
                }
                timeholder[edges.OtherNode] = timetemp;
                previousholder[edges.OtherNode] = backtemp;
            }
        }
    }
    std::stack <TNodeIndex> Pstack;
    TNodeID BNodeIndex = dest;
    Pstack.push(BNodeIndex);
    while (previousholder[BNodeIndex] != src) {
        Pstack.push(previousholder[BNodeIndex]);
        BNodeIndex = previousholder[BNodeIndex];
    }
    Pstack.push(src);
    while (!Pstack.empty()) {
        path.push_back(GetSortedNodeIDByIndex(Pstack.top()));
        Pstack.pop();
    }
    return timeholder[dest];
}
double CMapRouter::HaversineDistance(double lat1, double lon1, double lat2, double lon2){
    auto DegreesToRadians = [](double deg){return M_PI * (deg) / 180.0;};
    double LatRad1 = DegreesToRadians(lat1);
    double LatRad2 = DegreesToRadians(lat2);
    double LonRad1 = DegreesToRadians(lon1);
    double LonRad2 = DegreesToRadians(lon2);
    double DeltaLat = LatRad2 - LatRad1;
    double DeltaLon = LonRad2 - LonRad1;
    double DeltaLatSin = sin(DeltaLat/2);
    double DeltaLonSin = sin(DeltaLon/2);
    double Computation = asin(sqrt(DeltaLatSin * DeltaLatSin + cos(LatRad1) * cos(LatRad2) * DeltaLonSin * DeltaLonSin));
    const double EarthRadiusMiles = 3959.88;

    return 2 * EarthRadiusMiles * Computation;
}

double CMapRouter::CalculateBearing(double lat1, double lon1,double lat2, double lon2){
    auto DegreesToRadians = [](double deg){return M_PI * (deg) / 180.0;};
    auto RadiansToDegrees = [](double rad){return 180.0 * (rad) / M_PI;};
    double LatRad1 = DegreesToRadians(lat1);
    double LatRad2 = DegreesToRadians(lat2);
    double LonRad1 = DegreesToRadians(lon1);
    double LonRad2 = DegreesToRadians(lon2);
    double X = cos(LatRad2)*sin(LonRad2-LonRad1);
    double Y = cos(LatRad1)*sin(LatRad2)-sin(LatRad1)*cos(LatRad2)*cos(LonRad2-LonRad1);
    return RadiansToDegrees(atan2(X,Y));
}

bool CMapRouter::LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes){
    CXMLReader Reader(osm);
    
    SXMLEntity TempEntity;
    
    Reader.ReadEntity(TempEntity);
    std::cout<<"@ line: "<<__LINE__ <<std::endl;

    while (!Reader.End()) {
        Reader.ReadEntity(TempEntity);
        if (TempEntity.DType == SXMLEntity::EType::StartElement) {
            if (TempEntity.DNameData == "node") {
                std::cout<<"@ line: "<<__LINE__<<"node loop" <<std::endl;
                TNodeID TempID = std::stoul(TempEntity.AttributeValue("id"));
                double Templat = std::stod(TempEntity.AttributeValue("lat"));
                double Templong = std::stod(TempEntity.AttributeValue("lon"));
                Node TempNode;
                TempNode.NodeID = TempID;
                TempNode.TLat = Templat;
                TempNode.TLong = Templong;
               // TempNode.mode = "Walk";
                NodeTranslation[TempID] = Nodes.size();
                Nodes.push_back(TempNode);
                SortedIDs.push_back(TempID);
            }
            else if (TempEntity.DNameData == "way") {
                bool OneWay = false;
                std::vector<TNodeIndex> WayNode;
                double SpeedLimit = 25.0;
                while (!Reader.End()) {
                    Reader.ReadEntity(TempEntity,true);
                    if (TempEntity.DType == SXMLEntity::EType :: StartElement){
                        if(TempEntity.DNameData== "nd"){
                            std::cout<<"@ line: "<<__LINE__<< "inside nd loop"<<std::endl;
                            auto lookup = NodeTranslation.find(std::stoul(TempEntity.AttributeValue("ref")));
                            if(NodeTranslation.end() != lookup){
                                WayNode.push_back(lookup->second);
                            }
                        }
                        else if (TempEntity.DNameData== "tag"){
                            std::cout<<"@ line: "<<__LINE__<<"inside tag loop"<<std::endl;
                            auto key = TempEntity.AttributeValue("k");
                            auto value = TempEntity.AttributeValue("v");
                            if (key =="maxspeed"){
                                SpeedLimit = std::stod (value);
                                std::cout<<"@ line: "<<SpeedLimit << "mph" <<std::endl;
                            }
                            else if (key == "oneway"){
                                OneWay = value == "yes";
                            }
                        }
                    }
                    if (TempEntity.DType == SXMLEntity::EType::EndElement){
                        if (TempEntity.DNameData == "way"){
                            break;
                        }
                    }
                }
                for (int i = 0; i < WayNode.size()-1; i++ ){
                    Edge newEdge;
                    newEdge.OtherNode = WayNode[i+1];
                    double dist = HaversineDistance(Nodes[WayNode[i]].TLat, Nodes[WayNode[i]].TLong,
                                                      Nodes[WayNode[i+1]].TLat, Nodes[WayNode[i+1]].TLong);
                    newEdge.distance = dist;
                    newEdge.speedlimit = SpeedLimit;
                    std::cout << SpeedLimit << "this is speed limit"<<std::endl;
                    newEdge.time = dist/3.00000;
                    Nodes[WayNode[i]].Edges.push_back(newEdge);
                    if (!OneWay){
                        Edge newEdge1;
                        newEdge1.OtherNode = WayNode[i];
                        newEdge1.distance = newEdge.distance;
                        newEdge1.speedlimit = newEdge.speedlimit;
                        newEdge1.time = newEdge.time;
                        Nodes[WayNode[i+1]].Edges.push_back(newEdge1);
                    }
                }
                WayNode.clear();
            }
        }
    }
    std::sort (SortedIDs.begin(),SortedIDs.end());
    CCSVReader Reader1(stops);
    std::vector <std::string> readingVector;
    Reader1.ReadRow(readingVector);
    while(!Reader1.End()){
        Reader1.ReadRow(readingVector);
        auto lookup1 = NodeTranslation.find(std::stoul(readingVector[1]));
        if(NodeTranslation.end() != lookup1) {
            StopsMaps[std::stoul(readingVector[0])] = std::stoul(readingVector[1]);
            NodeIDtoStopID[std::stoul(readingVector[1])] = std::stoul(readingVector[0]);
        }
    }
    CCSVReader Reader2(routes);
    std::vector <std::string> readingVector1;
    Reader2.ReadRow(readingVector1);
    while(!Reader2.End()){
        readingVector1.clear();
        Reader2.ReadRow(readingVector1);
        auto lookup2 = StopsMaps.find(std::stoul(readingVector1[1]));
        if (StopsMaps.end() != lookup2){
            if (BusRoutesmap.find(readingVector1[0]) != BusRoutesmap.end()){
                std::cout << std::stoul(readingVector1[1])<<std::endl;
                BusRoutesmap.find(readingVector1[0])->second.push_back(std::stoul(readingVector1[1]));
            }else{
                routenames.push_back(readingVector1[0]);
                std::vector <TStopID> tempvector;
                tempvector.push_back(std::stoul(readingVector1[1]));
                BusRoutesmap[readingVector1[0]] = tempvector;
            }
        }
    }
    for (auto &nodes2 : Nodes){
        for (auto edg: nodes2.Edges){
            BusEdge edge4;
            edge4.time  = edg.time;
            edge4.vehicle = "Walk";
            edge4.Otherbusnode = edg.OtherNode;
            edge4.busname = "Walk";
            nodes2.Busedge.push_back(edge4);
        }
    }
    for (auto  ch : routenames) {
        auto &vectorofstop = BusRoutesmap.find(ch)->second;
        std::cout << "@ line: " << __LINE__ << std::endl;
        for (int o = 0; o < vectorofstop.size() - 1; o++) {
            auto ider = StopsMaps.find(vectorofstop[o])->second;
            auto otherid = StopsMaps.find(vectorofstop[o + 1])->second;
            BusEdge bus;
            bus.vehicle = "bus";
            bus.busname = ch;
            auto nodeinde = NodeTranslation.find(ider)->second;
            auto othernodeindex = NodeTranslation.find(otherid)->second;
            bus.Otherbusnode = NodeTranslation.find(otherid)->second;
            std::vector <TNodeID> fun;
            int checker1 = 0;
            for (auto &edges : Nodes[nodeinde].Edges) {
                if (edges.OtherNode == othernodeindex) {
                    bus.time = edges.distance / edges.speedlimit;
                    checker1++;
                }
            }
            if (checker1 == 0) {
                std::cout<< "this is fincalling dijkstra on "<< ider<<" "<<otherid<<std::endl;
                double time = Dijsktras(nodeinde, othernodeindex, fun);
                bus.time = time;
                bus.path1 = fun;
            }
            Nodes[nodeinde].Busedge.push_back(bus);
        }
    }
    return true;
}

size_t CMapRouter::NodeCount() const{
    return SortedIDs.size();
}

CMapRouter::TNodeID CMapRouter::GetSortedNodeIDByIndex(size_t index) const{
    return SortedIDs[index];
}
CMapRouter::TLocation CMapRouter::GetSortedNodeLocationByIndex(size_t index) const{
    if (index >= SortedIDs.size()){
        return std::make_pair(180.0, 360.0);
    }
    else{
        auto tempIndex = NodeTranslation.find(SortedIDs[index])->second;
        auto lat = Nodes[tempIndex].TLat;
        auto longs = Nodes[tempIndex].TLong;
        return std::make_pair(lat,longs);
    }
}

CMapRouter::TLocation CMapRouter::GetNodeLocationByID(TNodeID nodeid) const{
    if (NodeTranslation.end() == NodeTranslation.find(nodeid)){
        return std::make_pair(180.0, 360.0);
    }
    else{
        auto tempIndex = NodeTranslation.find(nodeid)->second;
        auto lat = Nodes[tempIndex].TLat;
        auto longs = Nodes[tempIndex].TLong;
        return std::make_pair(lat,longs);
    }
}

CMapRouter::TNodeID CMapRouter::GetNodeIDByStopID(TStopID stopid) const{
    auto lookup = StopsMaps.find(stopid);
    if (lookup != StopsMaps.end()){
        return lookup->second;
    }
}
size_t CMapRouter::RouteCount() const{
    return BusRoutesmap.size();
}
std::string CMapRouter::GetSortedRouteNameByIndex(size_t index) const{
    auto iter = BusRoutesmap.begin();
    auto iter1 = std::next(iter, index);
    return iter1->first;
}
bool CMapRouter::GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops){
    auto lookup = BusRoutesmap.find(route);
    if (lookup == BusRoutesmap.end()){
        return false;
    }
    stops = lookup->second;
    return true;
}

double CMapRouter::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    // Your code HERE
    TNodeIndex srcindex= NodeTranslation.find(src)->second;
    TNodeID destindex= NodeTranslation.find(dest)->second;
    double k = Dijsktras(srcindex,destindex,path);
    return k;
}
double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path){
    int sizeofnodes = Nodes.size();
    std:: vector<double> timecheck;
    std:: vector<long> previous;
    std:: vector<long> edgesforbus;
    
    for (int i =0; i<sizeofnodes;i++) {
        timecheck.push_back(std::numeric_limits<double>::max());
        previous.push_back(-1);
    }
    auto CompareLambda = [&timecheck](TNodeIndex idx1, TNodeIndex idx2){ return timecheck[idx1] <timecheck[idx2];};
    
    TNodeIndex srcindex= NodeTranslation.find(src)->second;
    
    std::cout<< srcindex<<std::endl;
    TNodeIndex destindex= NodeTranslation.find(dest)->second;
    timecheck[srcindex] = 0;
    previous[srcindex] = srcindex;

    std:: vector <TNodeIndex> Heap;
    std::make_heap (Heap.begin(),Heap.end(), CompareLambda);

    Heap.push_back(srcindex);
    std::push_heap (Heap.begin(),Heap.end());
    int thirtycheck =0;
    while(!Heap.empty()){
        auto Cur = Heap.front();
        Heap.erase(Heap.begin());
        auto Node1 = Nodes[Cur];
        for (auto edges1 : Node1.Busedge){
            double alttime = timecheck[Cur]+ edges1.time;
            if (edges1.vehicle == "bus"){
                thirtycheck++;
                alttime = alttime + (30.0/3600.0);
            }
            if (alttime < timecheck[edges1.Otherbusnode]){
                if (timecheck[edges1.Otherbusnode] == std::numeric_limits<double>::max()) {
                    Heap.push_back(edges1.Otherbusnode);
                    std::push_heap(Heap.begin(), Heap.end());
                }
                timecheck[edges1.Otherbusnode] = alttime;
                previous[edges1.Otherbusnode] = Cur;
            }
        }
        if (Cur == destindex){
            break;
        }
    }
    if (timecheck[destindex] ==std::numeric_limits<double>::max() ){
        path.clear();
        return std::numeric_limits<double>::max();
    }
    
    std::stack <TNodeIndex> maker;
    std::vector <TNodeIndex> path1;
    std::vector<TPathStep> buser;

    maker.push(destindex);
    TNodeIndex prev = previous[destindex];
    while(prev != srcindex){
        maker.push(prev);
        prev = previous[prev];
    }
    maker.push(srcindex);
    while(!maker.empty()){
        
        path1.push_back((maker.top()));
        maker.pop();
    }
    
    std::vector <std::vector <std::string> > possiblepaths;

    for (int dev = 0; dev <path1.size()-1; dev++){
        TNodeIndex indexx = path1[dev];
        
        std::cout<< dev<< std::endl;
        std::vector <std::string> empt;
        for (auto edge1: Nodes[indexx].Busedge){
            std::cout<<Nodes[indexx].NodeID<<  edge1.Otherbusnode +1 << edge1.vehicle <<std::endl;
            if (edge1.Otherbusnode == path1[dev+1]){
                std::string topush;
                if (edge1.vehicle == "bus"){
                    topush = "Bus " + edge1.busname;
                }else{
                    topush = edge1.busname;
                }
                empt.push_back(topush);
            }
            std::sort(empt.begin(),empt.end());
        }
        possiblepaths.push_back(empt);
        empt.clear();
    }

    for (int k = 0; k< possiblepaths.size(); k++){
        std::cout<<"index of possible path " << k<<std::endl;
        for (auto i : possiblepaths[k]){
            std::cout<< i<< std:: endl;

        }
    }
    
    std::pair <std::string,TNodeID> pusher1("Walk",src);
    path.push_back(pusher1);
    for(int y = 0; y < possiblepaths.size(); y++){
        
        std::vector<std::string> vec = possiblepaths[y];
        bool done = false;

        if( vec[0]=="Walk"){
            TNodeID idx = Nodes[path1[y+1]].NodeID;
            std::pair <std::string,TNodeID> pusher("Walk",idx);
            path.push_back(pusher);
        }
        else {
            int checkj = 0;
            std::vector <std::string> holder(26);
            std::vector <TNodeID> nodeids;
            std::vector<std::string>::iterator it;
            it = std::set_union(possiblepaths[y].begin(), possiblepaths[y].end(), possiblepaths[y + 1].begin(),
                                possiblepaths[y + 1].end(),holder.begin());
            holder.resize(it - holder.begin());
            nodeids.push_back(Nodes[path1[y]].NodeID);
            checkj++;
            while ((holder.size() >= 0) and (y < possiblepaths.size())) {
                if( possiblepaths[y + 1][0]=="Walk"){
                    break;
                }
                std::vector <std::string> union1 = holder;
                holder.clear();
                std::vector <std::string> holder(26);
                std::vector<std::string>::iterator it;
                it = std::set_intersection(union1.begin(), union1.end(), possiblepaths[y + 1].begin(),possiblepaths[y + 1].end(),holder.begin());
                holder.resize(it - holder.begin());
                if (holder.size()==0){
                    holder = union1;
                    break;
                }
                nodeids.push_back(Nodes[path1[y+1]].NodeID);
                y++;
                checkj++;
            }
            for (int aq = 0; aq<checkj;aq++){
                std::pair <std::string,TNodeID> pusher(holder[0],nodeids[aq+1]);
                path.push_back(pusher);
            }
        }
    }
    
    std::vector < std::pair <std::string, TNodeID> > holding;
    
    int stop2 = path.size();
    for (int rr = 0; rr< stop2-1; rr++ ){
        TNodeIndex firstnodeind = NodeTranslation.find(std::get<1> (path[rr]))->second;
        TNodeIndex secondnodeind = NodeTranslation.find(std::get<1> (path[rr+1]))->second;
        for (auto kk:Nodes[firstnodeind].Busedge){
            if (kk.Otherbusnode == secondnodeind) {
                if (kk.path1.size() > 2) {
                    std::cout << "this is where size is greater than 2 on index " << rr << "of the path" << kk.busname
                              << std::endl;
                    for (int i = 1; i < kk.path1.size() - 1; i++) {
                        std::pair<std::string, TNodeID> pushing(std::get<0>(path[rr]), kk.path1[i]);
                        holding.push_back(pushing);
                    }
                    path.insert(path.begin() + rr + 1, holding.begin(), holding.end());
                    rr += holding.size();
                    holding.clear();
                }
            }
        }
    }
    return timecheck[destindex];
}
