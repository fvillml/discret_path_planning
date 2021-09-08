#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

#include "path_planning/Map.hpp"
#include "path_planning/Planner.hpp"

int main() {
    // Instantiate map and planner objects
    std::string thisFile = __FILE__;
    std::string mapsFolder = thisFile.substr(0, thisFile.find("from_map.cpp")) + "../maps/";
    std::string mapFile = mapsFolder + "map.txt";
    std::cout << "Going to use map at " << mapFile << std::endl;
    int width { 150 }, height { 300 };
    Map map { width, height, mapFile };
    Planner::Point start { 230, 145 };
    Planner::Point goal { 60, 50 };
    int cost = 1;
    Planner planner { start, goal, cost };

    std::cout << "Searching....\n";
    auto heuristic = map.getHeuristic(goal, Map::Heuristic::MANHATTAN);
    planner.search(map, heuristic, false);

    std::string imageFile = mapsFolder + "path.png";
    planner.savePathToPng(imageFile, map);

    return 0;
}