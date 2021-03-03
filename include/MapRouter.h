#ifndef MAPROUTER_H
#define MAPROUTER_H

#include <vector>
#include <istream>
#include <map>

class CMapRouter{
    public:
        //user gives thede two node id
        using TNodeID = unsigned long;
        using TStopID = unsigned long;
        using TLocation = std::pair<double, double>;
        using TPathStep = std::pair<std::string, TNodeID>;
        using TNodeIndex = int;
        using TRoutes = std::string;

        static const TNodeID InvalidNodeID;
    private:
        struct Edge{
            TNodeIndex OtherNode;
            double time;
            double distance;
            double speedlimit;

        };
         struct BusEdge{
            double time;
            std::string vehicle;
            std::vector <TNodeID> path1;
            TNodeIndex Otherbusnode;
            TRoutes busname;
        };
        struct Node{
            TNodeID NodeID;
            double TLat;
            double TLong;
            std:: vector<Edge> Edges;
            std::vector <BusEdge> Busedge;
        };

        std::vector <Node> Nodes;
        std::vector <TNodeID> SortedIDs;
        std::map <TNodeID, TNodeIndex> NodeTranslation;
        std::map <TStopID, TNodeID> StopsMaps;
        std::map <TNodeID, TStopID> NodeIDtoStopID;
        std::map <TRoutes,std::vector<TStopID> > BusRoutesmap;
        std::vector<TRoutes> routenames;

    public:
        CMapRouter();
        ~CMapRouter();
        //calc distance in two lat and long in miles
        static double HaversineDistance(double lat1, double lon1, double lat2, double lon2);
        static double CalculateBearing(double lat1, double lon1,double lat2, double lon2);
        //take in input stream- parse it in and load it in
        bool LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes);
        //done
        size_t NodeCount() const;// can get same nodes regardless of implemntation
        //done
        TNodeID GetSortedNodeIDByIndex(size_t index) const;
        //done
        TLocation GetSortedNodeLocationByIndex(size_t index) const;
        //done
        TLocation GetNodeLocationByID(TNodeID nodeid) const;
        double Dijsktras(TNodeIndex src, TNodeIndex dest,std::vector<TNodeID>&path);

        TNodeID GetNodeIDByStopID(TStopID stopid) const;
        size_t RouteCount() const;
        std::string GetSortedRouteNameByIndex(size_t index) const;
        bool GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops);

        double FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path);
        double FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path);
};

#endif

