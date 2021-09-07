#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

#include "path_planning/Map.hpp"
#include "path_planning/Planner.hpp"

int main() {
    // Instantiate map and planner objects
    int width { 6 }, height { 5 };
    Map map { 6, 5 };
    Planner::Point start { 0, 0 };
    Planner::Point goal { height - 1, width - 1 };
    int cost = 1;
    Planner planner { start, goal, cost };

    // Print Map
    std::cout << "Map:" << std::endl;
    Map::print2DVector(map.mGrid);
    std::cout << std::endl;

    // Print heurisitc
    std::cout << "Heuristic:" << std::endl;
    auto heuristic = map.getHeuristic(goal, Map::Heuristic::NONE);
    Map::print2DVector(heuristic);
    std::cout << std::endl;

    // Print characteristics
    std::cout << "Start: " << start[0] << "," << start[1] << std::endl;
    std::cout << "Goal: " << goal[0] << "," << goal[1] << std::endl;
    std::cout << "Cost: " << cost << std::endl;
    std::cout << "Robot Movements: ";
    planner.printArrows();
    std::cout << "Delta:" << std::endl;
    planner.printMovements();
    std::cout << std::endl;
    std::cout << "Going to search" << std::endl;
    planner.search(map, heuristic);

    return 0;
}