#include "path_planning/Map.hpp"

Map::Map(const int& width, const int& height, const std::string& mapPath)
    : mWidth(width)
    , mHeight(height) {
    if (!mapPath.empty()) {
        std::ifstream mapFile;
        mapFile.open(mapPath.c_str());
        if (!mapFile.is_open()) {
            std::cout << "Failed to open file at " << mapPath << std::endl;
            return;
        }
        mGrid = Vector2D<int>(mHeight, std::vector<int>(mWidth));
        for (int i = 0; i < mHeight; i++) {
            for (int j = 0; j < mWidth; j++) {
                double cell {};
                mapFile >> cell;
                mGrid[i][j] = cell >= 0 ? 1 : 0;
            }
        }
    }
}

Map::Vector2D<int> Map::getHeuristic(const std::vector<int>& goal, const Heuristic& heuristicType) const {
    Vector2D<int> heuristic(mHeight, std::vector<int>(mWidth, -1));
    if (heuristicType == Heuristic::MANHATTAN) {
        for (int i = 0; i < mHeight; i++) {
            for (int j = 0; j < mWidth; j++) {
                int xd = goal.at(0) - i;
                int yd = goal.at(1) - j;
                int d = abs(xd) + abs(yd);
                heuristic[i][j] = d;
            }
        }
    } else {
        heuristic = Vector2D<int>(mHeight, std::vector<int>(mWidth, 1));
    }
    return heuristic;
}

bool Map::isFree(const int& x, const int& y) const {
    return x >= 0 && x < mHeight && y >= 0 && y < mWidth && mGrid.at(x).at(y) == 0;
}

int Map::getHeight() const {
    return mHeight;
}

int Map::getWidth() const {
    return mWidth;
}

void Map::printMap() const {
    print2DVector(mGrid);
}
