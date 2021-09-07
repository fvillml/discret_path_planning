#pragma once

#include <vector>
#include <set>
#include <functional>

#include "path_planning/Map.hpp"

class Planner {
public:
    typedef std::vector<int> Point;

    Planner(const Point &currentPos, const Point &goal, const int &cost)
        : mStart(currentPos)
        , mGoal(goal)
        , mCost(cost) {
    }

    void printMovements() {
        Map::print2DVector(mMovements);
    }

    void printArrows() {
        for (const auto &arrow : mMovementArrows) {
            std::cout << arrow << " ";
        }
        std::cout << std::endl;
    }

    void search(Map map, const Map::Vector2D<int> & heuristic) {
        Map::Vector2D<char> path(map.getHeight(), std::vector<char>(map.getWidth(), '-'));
        static auto currentPos = mStart;
        int g { 0 };
        int f = heuristic[currentPos[0]][currentPos[1]] + g;
        static Map::Vector2D<int> expansion(map.getHeight(), std::vector<int>(map.getWidth(), -1));
        static Map::Vector2D<int> actions(map.getHeight(), std::vector<int>(map.getWidth(), -1));
        static std::set<std::vector<int>> alreadyVisited { { currentPos } };
        static Map::Vector2D<int> frontier { { f, g, currentPos[0], currentPos[1] } };
        static int exp = 0;

        if (frontier.empty()) {
            std::cout << "Failed to find goal" << std::endl;
            return;
        }

        std::cout << "Expansion #" << exp << std::endl;
        std::cout << "Open List: \n";
        std::sort(frontier.begin(), frontier.end(), std::greater<std::vector<int>>());
        Map::print2DVector(frontier);

        f = frontier.back()[0];
        g = frontier.back()[1];
        currentPos[0] = frontier.back()[2];
        currentPos[1] = frontier.back()[3];
        expansion[currentPos[0]][currentPos[1]] = exp;
        frontier.pop_back();

        std::cout << "Cell Picked [" << f << " " << g << " " << currentPos[0] << " " << currentPos[1] << "]"
                  << std::endl;
        std::cout << std::endl;

        if (currentPos == mGoal) {
            Map::print2DVector(expansion);
            std::cout << std::endl;
            getPath(path, actions);
            Map::print2DVector(path);
            return;
        }

        int pos = 0;
        for (auto mov : mMovements) {
            int x = currentPos[0] + mov[0];
            int y = currentPos[1] + mov[1];
            if (map.isFree(x, y)) {
                // check if already visited
                if (alreadyVisited.insert({ x, y }).second) {
                    int newg = g + 1;
                    int newf = heuristic[x][y] + newg;
                    frontier.insert(frontier.begin(), { newf, newg, x, y });
                    actions[x][y] = pos;
                }
            }
            pos++;
        }

        exp++;
        search(map, heuristic);
    }

private:
    void getPath(Map::Vector2D<char> &path, Map::Vector2D<int> actions) {
        auto currentPos = mGoal;
        path[currentPos[0]][currentPos[1]] = '*';
        while (currentPos != mStart) {
            int mov = actions[currentPos[0]][currentPos[1]];
            currentPos[0] = currentPos[0] - mMovements[mov][0];
            currentPos[1] = currentPos[1] - mMovements[mov][1];
            path[currentPos[0]][currentPos[1]] = mMovementArrows[mov];
        }
    }
    Point mStart;
    Point mGoal;
    int mCost;
    Map::Vector2D<int> mMovements { { -1, 0 }, { 0, -1 }, { 1, 0 }, { 0, 1 } };
    std::vector<char> mMovementArrows { '^', '<', 'v', '>' };
};