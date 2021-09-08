#pragma once

#include <vector>
#include <iostream>
#include <fstream>

#include "matplotlibcpp.h"

class Map {
public:
    template <typename T>
    using Vector2D = std::vector<std::vector<T>>;

    enum class Heuristic { MANHATTAN, EUCLIDEAN, CHEBYSHEV, NONE };

    Map(const int& width, const int& height, const std::string& mapPath = "");

    Vector2D<int> getHeuristic(const std::vector<int>& goal, const Heuristic& heuristicType) const;

    bool isFree(const int& x, const int& y) const;

    int getHeight() const;

    int getWidth() const;

    void printMap() const;

    template <typename T>
    static void print2DVector(const Vector2D<T>& v2d) {
        for (const auto& v : v2d) {
            for (const auto& e : v) {
                std::cout << e << " ";
            }
            std::cout << std::endl;
        }
    }

private:
    const int mWidth;
    const int mHeight;

    Vector2D<int> mGrid {
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 1, 0 },
    };
};