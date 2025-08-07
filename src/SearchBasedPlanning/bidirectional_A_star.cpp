#include "bidirectional_A_star.h"

#include <cstdio>

BidirectionalAStar::BidirectionalAStar() {}

BidirectionalAStar::~BidirectionalAStar() {}

void BidirectionalAStar::init(Coordinate start, Coordinate goal, int type) {
    start_ = start;
    goal_ = goal;
    heuristic_type_ = type;
}

void BidirectionalAStar::searching(std::vector<Coordinate>& path,
                                   std::vector<Coordinate>& visited_order_fore,
                                   std::vector<Coordinate>& visited_order_back,
                                   const float* g) {
    // clang-format off
    std::vector<std::pair<int, int>> dirs = 
               { {-1, -1}, { 0, -1}, { 1, -1}, 
                 {-1,  0},           { 1,  0}, 
                 {-1,  1}, { 0,  1}, { 1,  1} };
    // clang-format on

    path.clear();
    visited_order_fore.clear();
    visited_order_back.clear();

    int maze_size = x_range_ * y_range_;
    Coordinate* parent_fore = new Coordinate[maze_size];
    Coordinate* parent_back = new Coordinate[maze_size];
    bool* visited_fore = new bool[maze_size];
    bool* visited_back = new bool[maze_size];
    float* g_fore = new float[maze_size];
    float* g_back = new float[maze_size];

    memset(parent_fore, 0xFF, sizeof(Coordinate) * maze_size);
    memset(parent_back, 0xFF, sizeof(Coordinate) * maze_size);
    memset(visited_fore, false, sizeof(bool) * maze_size);
    memset(visited_back, false, sizeof(bool) * maze_size);
    for (int y = 0; y < y_range_; y++) {
        for (int x = 0; x < x_range_; x++) {
            float cost = g[y * x_range_ + x];
            g_fore[y * x_range_ + x] = cost;
            g_back[y * x_range_ + x] = cost;
        }
    }

    parent_fore[start_.y * x_range_ + start_.x] = start_;
    parent_back[goal_.y * x_range_ + goal_.x] = goal_;
    g_fore[start_.y * x_range_ + start_.x] = 0.f;
    g_fore[goal_.y * x_range_ + goal_.x] = MAX_COST;
    g_back[goal_.y * x_range_ + goal_.x] = 0.f;
    g_back[start_.y * x_range_ + start_.x] = MAX_COST;

    std::unordered_set<Coordinate, CoordinateHash, CoordinateEqual> set_fore;
    std::unordered_set<Coordinate, CoordinateHash, CoordinateEqual> set_back;

    std::priority_queue<CoordinatePair, std::vector<CoordinatePair>,
                        std::greater<CoordinatePair>>
        pq_fore;
    std::priority_queue<CoordinatePair, std::vector<CoordinatePair>,
                        std::greater<CoordinatePair>>
        pq_back;

    pq_fore.push(CoordinatePair(calc_f_value(start_, g_fore), start_));
    pq_back.push(CoordinatePair(calc_f_value(goal_, g_back), goal_));

    Coordinate meet_point = start_;

    while (!pq_fore.empty() && !pq_back.empty()) {
        Coordinate cur_point_fore = pq_fore.top().second;
        pq_fore.pop();

        if (set_back.count(cur_point_fore)) {
            meet_point = cur_point_fore;
            break;
        }

        visited_fore[cur_point_fore.y * x_range_ + cur_point_fore.x] = true;
        visited_order_fore.push_back(cur_point_fore);

        for (auto dir : dirs) {
            int new_x = cur_point_fore.x + dir.first;
            int new_y = cur_point_fore.y + dir.second;
            if (new_x >= 0 && new_x < x_range_ && new_y >= 0 &&
                new_y < y_range_) {
                Coordinate new_point = Coordinate(new_x, new_y);
                bool is_visited =
                    visited_fore[new_point.y * x_range_ + new_point.x];
                if (is_visited)
                    continue;

                float prev_cost =
                    g_fore[cur_point_fore.y * x_range_ + cur_point_fore.x];
                float new_cost =
                    prev_cost + calc_cost(cur_point_fore, new_point, g_fore);
                float cost = g_fore[new_point.y * x_range_ + new_point.x];
                if (new_cost < cost) {
                    g_fore[new_point.y * x_range_ + new_point.x] = new_cost;
                    parent_fore[new_point.y * x_range_ + new_point.x] =
                        cur_point_fore;
                    set_fore.insert(cur_point_fore);

                    float priority = calc_f_value(new_point, g_fore);
                    pq_fore.push(CoordinatePair(priority, new_point));
                }
            }
        }

        // solve backward-search
        Coordinate cur_point_back = pq_back.top().second;
        pq_back.pop();

        if (set_fore.count(cur_point_back)) {
            meet_point = cur_point_back;
            break;
        }

        visited_back[cur_point_back.y * x_range_ + cur_point_back.x] = true;
        visited_order_back.push_back(cur_point_back);

        for (auto dir : dirs) {
            int new_x = cur_point_back.x + dir.first;
            int new_y = cur_point_back.y + dir.second;
            if (new_x >= 0 && new_x < x_range_ && new_y >= 0 &&
                new_y < y_range_) {
                Coordinate new_point = Coordinate(new_x, new_y);
                bool is_visited =
                    visited_back[new_point.y * x_range_ + new_point.x];
                if (is_visited)
                    continue;

                float prev_cost =
                    g_back[cur_point_back.y * x_range_ + cur_point_back.x];
                float new_cost =
                    prev_cost + calc_cost(cur_point_back, new_point, g_back);
                float cost = g_back[new_point.y * x_range_ + new_point.x];
                if (new_cost < cost) {
                    g_back[new_point.y * x_range_ + new_point.x] = new_cost;
                    parent_back[new_point.y * x_range_ + new_point.x] =
                        cur_point_back;
                    set_back.insert(cur_point_back);

                    float priority = calc_f_value(new_point, g_back);
                    pq_back.push(CoordinatePair(priority, new_point));
                }
            }
        }
    }
    std::cout << "search path done... " << std::endl;

    extract_path(path, meet_point, parent_fore, parent_back);
    std::cout << "extract path done... " << std::endl;
    std::cout << "total length is " << path.size() << ". " << std::endl;

    delete[] parent_fore;
    delete[] parent_back;
    delete[] visited_fore;
    delete[] visited_back;
    delete[] g_fore;
    delete[] g_back;
}

void BidirectionalAStar::extract_path(std::vector<Coordinate>& path,
                                      const Coordinate meet_point,
                                      const Coordinate* parent_fore,
                                      const Coordinate* parent_back) {
    std::vector<Coordinate> path_fore;
    path_fore.push_back(meet_point);
    Coordinate point = meet_point;

    while (true) {
        point = parent_fore[point.y * x_range_ + point.x];
        path_fore.push_back(point);
        if (point == start_)
            break;
    }
    std::vector<Coordinate> path_back;
    point = meet_point;

    while (true) {
        point = parent_back[point.y * x_range_ + point.x];
        path_back.push_back(point);
        if (point == goal_)
            break;
    }

    for (int i = path_fore.size() - 1; i >= 0; i--) {
        path.push_back(path_fore[i]);
    }

    for (int i = 0; i < path_back.size(); i++) {
        path.push_back(path_back[i]);
    }
}

float BidirectionalAStar::heuristic(const Coordinate& p) {
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

float BidirectionalAStar::calc_f_value(const Coordinate& p, float* g) {
    float cost = g[p.y * x_range_ + p.x];
    return cost + heuristic(p);
}

int main() {
    printf("Bidirectional A Star ... \n");
    Coordinate start = Coordinate(5, 5);
    Coordinate goal = Coordinate(45, 25);

    Maze maze = Maze();
    maze.init(start, goal);

    std::vector<Coordinate> visited_order_1;
    std::vector<Coordinate> visited_order_2;
    std::vector<Coordinate> path;

    BidirectionalAStar searcher = BidirectionalAStar();
    searcher.init(start, goal, EUCLIDEAN);
    searcher.searching(path, visited_order_1, visited_order_2, maze.g);

    maze.animation(path, visited_order_1, visited_order_2);

    return 0;
}