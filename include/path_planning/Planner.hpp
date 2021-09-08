#pragma once

#include <vector>
#include <set>
#include <functional>
#include <algorithm>

#include "path_planning/Map.hpp"

class Planner {
public:
    typedef std::vector<int> Point;

    Planner(const Point &currentPos, const Point &goal, const int &cost);

    void printMovements() const;

    void printArrows() const;

    void search(Map map, const Map::Vector2D<int> &heuristic, bool verbose = true);

    void savePathToPng(const std::string &imagePath, const Map &map);

    Point getGoal() const;

    Point getStart() const;

private:
    void setPath(Map::Vector2D<char> &charMap, Map::Vector2D<int> actions);

    Point mStart;
    Point mGoal;
    Map::Vector2D<int> mPath;
    int mCost;
    Map::Vector2D<int> mMovements { { -1, 0 }, { 0, -1 }, { 1, 0 }, { 0, 1 } };
    std::vector<char> mMovementArrows { '^', '<', 'v', '>' };
};