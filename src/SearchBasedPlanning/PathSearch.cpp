#include "PathSearch.h"

void PathSearch::init(Coordinate start, Coordinate goal) {
    start_ = start;
    goal_ = goal;
}

void PathSearch::searching(std::vector<Coordinate>& path,
                           std::vector<Coordinate>& visited_order, float* g) {
    std::cout << "This is a based class if search based planning. "
              << std::endl;
    visited_order.clear();
    path.clear();
}

std::vector<Coordinate> PathSearch::get_neighbors(const Coordinate& p,
                                                  const float* g) {
    // clang-format off
    std::vector<std::pair<int, int>> dirs = 
               { {-1, -1}, { 0, -1}, { 1, -1}, 
                 {-1,  0},           { 1,  0}, 
                 {-1,  1}, { 0,  1}, { 1,  1} };
    // std::vector<std::pair<int, int>> dirs = 
    //            { {-1,  0}, {-1,  1}, { 0,  1}, 
    //              { 1,  1},           { 1,  0}, 
    //              { 1, -1}, { 0, -1}, {-1, -1} };
    // clang-format on
    std::vector<Coordinate> neighbors;
    for (auto dir : dirs) {
        int new_x = p.x + dir.first;
        int new_y = p.y + dir.second;
        if (new_x >= 0 && new_x < x_range_ && new_y >= 0 && new_y < y_range_) {
            Coordinate new_point = Coordinate(new_x, new_y);
            int new_index = new_point.y * x_range_ + new_point.x;
            bool is_obstacle = g[new_index] == OBS_COST ? true : false;
            if (!is_obstacle)
                neighbors.push_back(new_point);
        }
    }
    return neighbors;
}

float PathSearch::calc_cost(const Coordinate& p1, const Coordinate& p2,
                            const float* g) {
    if (is_collision(p1, p2, g))
        return MAX_COST;

    float x = static_cast<float>(p2.x - p1.x);
    float y = static_cast<float>(p2.y - p1.y);
    float cost = std::hypot(x, y);

    return cost;
}

bool PathSearch::is_collision(const Coordinate& p1, const Coordinate& p2,
                              const float* g) {
    int cx1 = p1.x;
    int cy1 = p1.y;
    int cx2 = p2.x;
    int cy2 = p2.y;

    if (g[cy1 * x_range_ + cx1] == OBS_COST ||
        g[cy2 * x_range_ + cx2] == OBS_COST)
        return true;

    Coordinate s1, s2;
    if (cx1 != cx2 && cy1 != cy2) {
        if (cx2 - cx1 == cy1 - cy2) {
            s1 = Coordinate(std::min(cx1, cx2), std::min(cy1, cy2));
            s2 = Coordinate(std::max(cx1, cx2), std::max(cy1, cy2));
        } else {
            s1 = Coordinate(std::min(cx1, cx2), std::max(cy1, cy2));
            s2 = Coordinate(std::max(cx1, cx2), std::min(cy1, cy2));
        }

        if (g[s1.y * x_range_ + s1.x] == OBS_COST ||
            g[s2.y * x_range_ + s2.y] == OBS_COST) {
            return true;
        }
    }

    return false;
}

void PathSearch::extract_path(std::vector<Coordinate>& path,
                              Coordinate* parent) {
    // calculate path
    Coordinate cur_point = goal_;
    path.push_back(cur_point);
    while (cur_point != start_) {
        Coordinate prev_point = parent[cur_point.y * x_range_ + cur_point.x];
        if (prev_point.x == -1 || prev_point.y == -1)
            break;
        cur_point = prev_point;
        path.push_back(cur_point);
    }
}