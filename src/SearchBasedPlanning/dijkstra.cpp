#include "dijkstra.h"

#include <cstdio>

Dijkstra::Dijkstra() {}

Dijkstra::~Dijkstra() {}

void Dijkstra::init(Coordinate start, Coordinate goal) {
    start_ = start;
    goal_ = goal;
}

void Dijkstra::searching(std::vector<Coordinate>& path,
                      std::vector<Coordinate>& visited_order, float* g) {
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
    memset(parent, 0xFF, sizeof(Coordinate) * maze_size);
    memset(visited, false, sizeof(bool) * maze_size);

    parent[start_.y * x_range_ + start_.x] = start_;
    g[start_.y * x_range_ + start_.x] = 0.f;
    g[goal_.y * x_range_ + goal_.x] = FLT_MAX;

    std::priority_queue<CoordinatePair, std::vector<CoordinatePair>,
                        std::greater<CoordinatePair>>
        pq;

    pq.push(CoordinatePair(0.f, start_));

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

                float prev_cost = g[cur.y * x_range_ + cur.x];
                float new_cost = prev_cost + calc_cost(cur, new_point, g);
                float cost = g[new_point.y * x_range_ + new_point.x];
                if (is_visited)
                    continue;
                if (new_cost < cost) {
                    g[new_point.y * x_range_ + new_point.x] = new_cost;
                    parent[new_point.y * x_range_ + new_point.x] = cur;
                    
                    pq.push(CoordinatePair(new_cost, new_point));
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
}

int main() {
    printf("A Star ... \n");
    Coordinate start = Coordinate(5, 5);
    Coordinate goal = Coordinate(45, 25);

    Maze maze = Maze();
    maze.init(start, goal);

    std::vector<Coordinate> visited_order;
    std::vector<Coordinate> path;

    Dijkstra searcher = Dijkstra();
    searcher.init(start, goal);
    searcher.searching(path, visited_order, maze.g);

    maze.animation(path, visited_order);

    return 0;
}