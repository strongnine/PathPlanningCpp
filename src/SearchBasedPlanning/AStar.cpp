#include "AStar.h"

#include <cstdio>

AStar::AStar() {}

AStar::~AStar() {}

void AStar::init(Coordinate start, Coordinate goal, int type) {
    start_ = start;
    goal_ = goal;
    heuristic_type_ = type;
}

void AStar::searching(std::vector<Coordinate>& path,
                      std::vector<Coordinate>& visited_order, const float* g) {
    // clang-format off
    std::vector<std::pair<int, int>> dirs = 
               { {-1, -1}, { 0, -1}, { 1, -1}, 
                 {-1,  0},           { 1,  0}, 
                 {-1,  1}, { 0,  1}, { 1,  1} };
    // clang-format on

    visited_order.clear();
    path.clear();

    int maze_size = x_range_ * y_range_;
    Coordinate* parent = new Coordinate[maze_size];
    bool* visited = new bool[maze_size];
    float* g_c = new float[maze_size];
    memset(parent, 0xFF, sizeof(Coordinate) * maze_size);
    memset(visited, false, sizeof(bool) * maze_size);
    memcpy(g_c, g, sizeof(float) * maze_size);

    parent[start_.y * x_range_ + start_.x] = start_;
    g_c[start_.y * x_range_ + start_.x] = 0.f;
    g_c[goal_.y * x_range_ + goal_.x] = FLT_MAX;

    std::priority_queue<CoordinatePair, std::vector<CoordinatePair>,
                        std::greater<CoordinatePair>>
        pq;

    pq.push(CoordinatePair(calc_f_value(start_, g_c, e_), start_));

    while (!pq.empty()) {
        CoordinatePair cur_pair = pq.top();
        Coordinate cur = cur_pair.second;
        visited[cur.y * x_range_ + cur.x] = true;
        pq.pop();
        visited_order.push_back(cur);
        if (cur == goal_)
            break;

        for (auto dir : dirs) {
            int new_x = cur.x + dir.first;
            int new_y = cur.y + dir.second;
            if (new_x >= 0 && new_x < x_range_ && new_y >= 0 &&
                new_y < y_range_) {
                Coordinate new_point = Coordinate(new_x, new_y);
                bool is_visited = visited[new_point.y * x_range_ + new_point.x];
                if (is_visited)
                    continue;

                float prev_cost = g_c[cur.y * x_range_ + cur.x];
                float new_cost = prev_cost + calc_cost(cur, new_point, g_c);
                float cost = g_c[new_point.y * x_range_ + new_point.x];
                if (new_cost < cost) {
                    g_c[new_point.y * x_range_ + new_point.x] = new_cost;
                    parent[new_point.y * x_range_ + new_point.x] = cur;

                    float priority = calc_f_value(new_point, g_c, e_);
                    pq.push(CoordinatePair(priority, new_point));
                }
            }
        }
    }
    std::cout << "search path done... " << std::endl;

    extract_path(path, parent);
    std::cout << "extract path done... " << std::endl;
    std::cout << "total length is " << path.size() << ". " << std::endl;

    delete[] parent;
    delete[] visited;
    delete[] g_c;
}

void AStar::repeated_searching(std::vector<Coordinate>& path,
                               std::vector<Coordinate>& visited_order,
                               const float* g) {
    searching(path, visited_order, g);
    e_ -= 0.5f;
}

float AStar::heuristic(const Coordinate& p) {
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

float AStar::calc_f_value(const Coordinate& p, float* g, float e) {
    float cost = g[p.y * x_range_ + p.x];
    return cost + e * heuristic(p);
}

void AStar::set_e(float e) { e_ = e; }

float AStar::get_e() { return e_; }

int main() {
    printf("A Star ... \n");
    Coordinate start = Coordinate(5, 5);
    Coordinate goal = Coordinate(45, 25);

    Maze maze = Maze();
    maze.init(start, goal);

    std::vector<Coordinate> visited_order;
    std::vector<Coordinate> path;

    AStar searcher = AStar();
    searcher.init(start, goal, EUCLIDEAN);

#if 0
    searcher.searching(path, visited_order, maze.g);
    maze.animation(path, visited_order);
#else
    searcher.set_e(2.5f);
    while (searcher.get_e() >= 1.0f) {
        searcher.repeated_searching(path, visited_order, maze.g);
        maze.animation(path, visited_order);
    }
#endif
    return 0;
}