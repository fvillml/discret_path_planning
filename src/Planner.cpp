#include "path_planning/Planner.hpp"

Planner::Planner(const Point &currentPos, const Point &goal, const int &cost)
    : mStart(currentPos)
    , mGoal(goal)
    , mCost(cost) {
}

void Planner::printMovements() const {
    Map::print2DVector(mMovements);
}

void Planner::printArrows() const {
    for (const auto &arrow : mMovementArrows) {
        std::cout << arrow << " ";
    }
    std::cout << std::endl;
}

void Planner::search(Map map, const Map::Vector2D<int> &heuristic, bool verbose) {
    Map::Vector2D<char> charMap(map.getHeight(), std::vector<char>(map.getWidth(), '-'));
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

    std::sort(frontier.begin(), frontier.end(), std::greater<std::vector<int>>());
    if (verbose) {
        std::cout << "Expansion #" << exp << std::endl;
        std::cout << "Open List: \n";
        Map::print2DVector(frontier);
    }

    f = frontier.back()[0];
    g = frontier.back()[1];
    currentPos[0] = frontier.back()[2];
    currentPos[1] = frontier.back()[3];
    expansion[currentPos[0]][currentPos[1]] = exp;
    frontier.pop_back();

    if (verbose) {
        std::cout << "Cell Picked [" << f << " " << g << " " << currentPos[0] << " " << currentPos[1] << "]"
                  << std::endl
                  << std::endl;
    }

    if (currentPos == mGoal) {
        setPath(charMap, actions);
        if (verbose) {
            Map::print2DVector(expansion);
            std::cout << std::endl;
            Map::print2DVector(charMap);
        }
        std::cout << "Path found !\n";
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
    search(map, heuristic, verbose);
}

void Planner::savePathToPng(const std::string &imagePath, const Map &map) {
    // Graph Format
    matplotlibcpp::title("Path");
    matplotlibcpp::xlim(0, map.getHeight());
    matplotlibcpp::ylim(0, map.getWidth());

    // Draw every grid of the map:
    std::cout << "Plotting...\n";
    for (std::size_t x = 0; x < map.getHeight(); x++) {
        for (std::size_t y = 0; y < map.getWidth(); y++) {
            double xd { static_cast<double>(x) }, yd { static_cast<double>(y) };
            if (map.isFree(x, y)) {
                matplotlibcpp::plot({ xd }, { yd }, "r.");
            } else {
                matplotlibcpp::plot({ xd }, { yd }, "k.");
            }
        }
    }

    double goalx { static_cast<double>(mGoal.at(0)) };
    double goaly { static_cast<double>(mGoal.at(1)) };
    matplotlibcpp::plot({ goalx }, { goaly }, "bo");

    double startx { static_cast<double>(mStart.at(0)) };
    double starty { static_cast<double>(mStart.at(1)) };
    matplotlibcpp::plot({ startx }, { starty }, "b*");

    // TODO: Plot the robot path in blue color using a .
    for (const auto &point : mPath) {
        double xd = static_cast<double>(point.at(0));
        double yd = static_cast<double>(point.at(1));
        matplotlibcpp::plot({ xd }, { yd }, "b.");
    }

    // Save the image and close the plot
    std::cout << "Going to save image to " << imagePath << std::endl;
    matplotlibcpp::save(imagePath);
    matplotlibcpp::clf();
}

Planner::Point Planner::getGoal() const {
    return mGoal;
}

Planner::Point Planner::getStart() const {
    return mStart;
}

void Planner::setPath(Map::Vector2D<char> &charMap, Map::Vector2D<int> actions) {
    mPath.clear();
    auto currentPos = mGoal;
    charMap[currentPos[0]][currentPos[1]] = '*';
    while (currentPos != mStart) {
        int mov = actions[currentPos[0]][currentPos[1]];
        currentPos[0] = currentPos[0] - mMovements[mov][0];
        currentPos[1] = currentPos[1] - mMovements[mov][1];
        charMap[currentPos[0]][currentPos[1]] = mMovementArrows[mov];
        mPath.push_back({ currentPos[0], currentPos[1] });
    }
}