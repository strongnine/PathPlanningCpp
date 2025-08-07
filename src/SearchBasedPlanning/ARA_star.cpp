#include "ARA_star.h"

#include <cstdio>

bool operator<(const std::pair<Coordinate, float>& p1,
               const std::pair<Coordinate, float>& p2) {
    return p1.second < p2.second;
}

ARA_star::ARA_star() {}

ARA_star::~ARA_star() {
    SAFE_DELETE(parent_);
    SAFE_DELETE(g_);
}

void ARA_star::init(Coordinate start, Coordinate goal, int type) {
    start_ = start;
    goal_ = goal;
    heuristic_type_ = type;
}

void ARA_star::searching(std::vector<Coordinate>& path,
                         std::vector<Coordinate>& visited_order, float* g) {
    int maze_size = x_range_ * y_range_;
    if (!init_state_) {
        parent_ = new Coordinate[maze_size];
        visited_ = new bool[maze_size];
        g_ = new float[maze_size];
        memset(parent_, 0xFF, sizeof(Coordinate) * maze_size);
        memset(visited_, false, sizeof(bool) * maze_size);
        memcpy(g_, g, sizeof(float) * maze_size);

        parent_[start_.y * x_range_ + start_.x] = start_;
        g_[start_.y * x_range_ + start_.x] = 0.f;
        g_[goal_.y * x_range_ + goal_.x] = MAX_COST;
        open_set_[start_] = calc_f_value(start_, g_, e_);

        init_state_ = true;
        improve_path(path, visited_order, open_set_, incons_set_);
    } else {
        if (update_e(open_set_, incons_set_) > 1) {
            e_ -= 0.4f;
            // update incons_set to open_set
            for (auto item = incons_set_.begin(); item != incons_set_.end();
                 item++)
                open_set_[item->first] = item->second;
            for (auto item = open_set_.begin(); item != open_set_.end();
                 item++) {
                open_set_[item->first] = calc_f_value(item->first, g_, e_);
            }
            incons_set_.clear();
            memset(visited_, false, sizeof(bool) * maze_size);
            improve_path(path, visited_order, open_set_, incons_set_);
        } else {
            search_done_state_ = true;
            std::cout << "Search done... " << std::endl;
        }
    }
}

void ARA_star::improve_path(std::vector<Coordinate>& path,
                            std::vector<Coordinate>& visited_order,
                            std::map<Coordinate, float>& open_set,
                            std::map<Coordinate, float>& incons_set) {
    // clang-format off
    std::vector<std::pair<int, int>> dirs = 
               { {-1, -1}, { 0, -1}, { 1, -1}, 
                 {-1,  0},           { 1,  0}, 
                 {-1,  1}, { 0,  1}, { 1,  1} };
    // clang-format on
    path.clear();
    visited_order.clear();
    while (true) {
        auto top = std::min_element(open_set.begin(), open_set.end(),
                                    std::less<std::pair<Coordinate, float>>());
        Coordinate point = top->first;
        float f_min = top->second;

        if (calc_f_value(goal_, g_, e_) <= f_min) {
            break;
        }

        open_set.erase(point);
        visited_[point.y * x_range_ + point.x] = true;

        for (auto dir : dirs) {
            int new_x = point.x + dir.first;
            int new_y = point.y + dir.second;
            if (new_x >= 0 && new_x < x_range_ && new_y >= 0 &&
                new_y < y_range_) {
                Coordinate new_point = Coordinate(new_x, new_y);
                if (g_[new_point.y * x_range_ + new_point.x] == OBS_COST)
                    continue;
                bool is_visited =
                    visited_[new_point.y * x_range_ + new_point.x];
                if (is_visited) {
                    incons_set[new_point] = 0.f;
                    continue;
                }

                float prev_cost = g_[point.y * x_range_ + point.x];
                float new_cost = prev_cost + calc_cost(point, new_point, g_);
                float cost = g_[new_point.y * x_range_ + new_point.x];
                if (new_cost < cost) {
                    g_[new_point.y * x_range_ + new_point.x] = new_cost;
                    parent_[new_point.y * x_range_ + new_point.x] = point;

                    visited_order.push_back(new_point);
                    open_set[new_point] = calc_f_value(new_point, g_, e_);
                }
            }
        }
    }

    extract_path(path, parent_);
}

float ARA_star::heuristic(const Coordinate& p) {
    Coordinate goal = goal_;
    float res;
    if (heuristic_type_ == MANHATTAN) {
        int dist = std::abs(goal_.x - p.x) + std::abs(goal_.y - p.y);
        res = static_cast<float>(dist);
    } else {
        float lx = static_cast<float>(goal_.x - p.x);
        float ly = static_cast<float>(goal_.y - p.y);
        res = std::hypot(lx, ly);
    }
    return res;
}

float ARA_star::calc_f_value(const Coordinate& p, const float* g, float e) {
    float cost = g[p.y * x_range_ + p.x];
    return cost + e * heuristic(p);
}
float ARA_star::update_e(std::map<Coordinate, float>& open_set,
                         std::map<Coordinate, float>& incons_set) {
    float v = FLT_MAX;
    if (!open_set.empty()) {
        for (auto item = open_set.begin(); item != open_set.end(); item++) {
            Coordinate point = item->first;
            v = std::min(g_[point.y * x_range_ + point.x] + heuristic(point),
                         v);
        }
    }
    if (!incons_set.empty()) {
        for (auto item = incons_set.begin(); item != incons_set.end(); item++) {
            Coordinate point = item->first;
            v = std::min(g_[point.y * x_range_ + point.x] + heuristic(point),
                         v);
        }
    }
    float res = std::min(e_, g_[goal_.y * x_range_ + goal_.x] / v);
    return std::min(e_, g_[goal_.y * x_range_ + goal_.x] / v);
}

void ARA_star::set_e(float e) { e_ = e; }

float ARA_star::get_e() { return e_; }

bool ARA_star::is_search_done() { return search_done_state_; }

int main() {
    printf("Anytime repeated A_star ... \n");
    Coordinate start = Coordinate(5, 5);
    Coordinate goal = Coordinate(45, 25);

    Maze maze = Maze();
    maze.init(start, goal);

    std::vector<Coordinate> visited_order;
    std::vector<Coordinate> path;

    ARA_star searcher = ARA_star();
    searcher.init(start, goal, EUCLIDEAN);
    searcher.set_e(2.5f);

    while (!searcher.is_search_done()) {
        searcher.searching(path, visited_order, maze.g);
        maze.animation(path, visited_order);
    }

    return 0;
}