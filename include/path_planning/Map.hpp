#pragma once

#include <vector>
#include <iostream>

class Map {
public:
    template <typename T>
    using Vector2D = std::vector<std::vector<T>>;

    enum class Heuristic { MANHATTAN, EUCLIDEAN, CHEBYSHEV, NONE };

    Map(const int& width, const int& height)
        : mWidth(width)
        , mHeight(height) {
    }

    template <typename T>
    static void print2DVector(const Vector2D<T>& v2d) {
        for (const auto& v : v2d) {
            for (const auto& e : v) {
                std::cout << e << " ";
            }
            std::cout << std::endl;
        }
    }

    Vector2D<int> mGrid {
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 1, 0 },
    };

    Vector2D<int> getHeuristic(const std::vector<int>& goal, const Heuristic& heuristicType) {
        Vector2D<int> heuristic(mHeight, std::vector<int>(mWidth, -1));
        if (heuristicType == Heuristic::MANHATTAN) {
            for (int i = 0; i < mHeight; i++) {
                for (int j = 0; j < mWidth; j++) {
                    int xd = goal[0] - i;
                    int yd = goal[1] - j;
                    int d = abs(xd) + abs(yd);
                    heuristic[i][j] = d;
                }
            }
        } else {
            heuristic = Vector2D<int>(mHeight, std::vector<int>(mWidth, 1));
        }
        return heuristic;
    }

    bool isFree(const int& x, const int& y) {
        return x >= 0 && x < mHeight && y >= 0 && y < mWidth && mGrid[x][y] == 0;
    }

    int getHeight() {
        return mHeight;
    }

    int getWidth() {
        return mWidth;
    }

private:
    const int mWidth;
    const int mHeight;
};