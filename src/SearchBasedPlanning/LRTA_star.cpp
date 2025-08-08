#include "LRTA_star.h"

LRTA_star::LRTA_star() {}

LRTA_star::~LRTA_star() {
    SAFE_DELETE(parent_);
    SAFE_DELETE(visited_);
    SAFE_DELETE(g_);
    SAFE_DELETE(h_);
    SAFE_DELETE(h_prime_);
}

void LRTA_star::init(Coordinate start, Coordinate goal, int N, int type) {
    start_ = start;
    goal_ = goal;
    N_ = N;
    heuristic_type_ = type;

    int maze_size = x_range_ * y_range_;
    parent_ = new Coordinate[maze_size];
    visited_ = new bool[maze_size];
    g_ = new float[maze_size];
    h_ = new float[maze_size];
    h_prime_ = new float[maze_size];
    for (int y = 0; y < y_range_; y++) {
        for (int x = 0; x < x_range_; x++) {
            Coordinate point(x, y);
            h_[y * x_range_ + x] = heuristic(point);
            h_prime_[y * x_range_ + x] = FLT_MAX;
        }
    }
}

void LRTA_star::searching(
    std::vector<std::vector<Coordinate>>& path_iter,
    std::vector<std::vector<Coordinate>>& visited_order_iter, const float* g) {
    Coordinate cur_start = start_;
    int maze_size = x_range_ * y_range_;
    while (true) {
        std::vector<Coordinate> path;
        std::vector<Coordinate> visited_order;

        CoordinatePairQueue pq;
        bool is_found = A_star(path, visited_order, pq, g, cur_start, N_);
        if (is_found) {
            path_iter.push_back(path);
            visited_order_iter.push_back((visited_order));
            break;
        }

        update_h_table();
        extract_path_in_close_set(path, cur_start);

        path_iter.push_back(path);
        visited_order_iter.push_back((visited_order));
    }
}

bool LRTA_star::A_star(std::vector<Coordinate>& path,
                       std::vector<Coordinate>& visited_order,
                       CoordinatePairQueue& pq, const float* g,
                       const Coordinate cur_start, int N) {
    pq.push(CoordinatePair(heuristic(cur_start), cur_start));
    int maze_size = x_range_ * y_range_;
    memset(parent_, 0xFF, sizeof(Coordinate) * maze_size);
    memset(visited_, false, sizeof(bool) * maze_size);
    memcpy(g_, g, sizeof(float) * maze_size);
    g_[cur_start.y * x_range_ + cur_start.x] = 0.f;
    g_[goal_.y * x_range_ + goal_.x] = MAX_COST;

    parent_[cur_start.y * x_range_ + cur_start.x] = cur_start;
    int count = 0;
    while (!pq.empty()) {
        count++;
        Coordinate point = pq.top().second;
        pq.pop();
        visited_[point.y * x_range_ + point.x] = true;
        visited_order.push_back(point);

        if (point == goal_) {
            extract_path(path, cur_start, parent_);
            return true;
        }

        for (Coordinate new_point : get_neighbors(point, g_)) {
            int new_index = new_point.y * x_range_ + new_point.x;
            bool is_visited = visited_[new_index];
            if (!is_visited) {
                float prev_cost = g_[point.y * x_range_ + point.x];
                float new_cost = prev_cost + calc_cost(point, new_point, g_);
                float cost = g_[new_index];
                if (new_cost < cost) {
                    g_[new_index] = new_cost;
                    parent_[new_index] = point;

                    float priority = g_[new_index] + h_[new_index];
                    pq.push(CoordinatePair(priority, new_point));
                }
            }
        }
        if (count == N)  // expand needed CLOSED nodes
            break;
    }

    return false;
}

void LRTA_star::update_h_table() {
    int maze_size = x_range_ * y_range_;
    float* h_prime_copy = new float[maze_size];
    for (int i = 0; i < maze_size; i++) {
        if (visited_[i]) {
            h_prime_[i] = MAX_COST * 1.5f;
        } else {
            h_prime_[i] = MAX_COST;
        }
    }
    while (true) {
        memcpy(h_prime_copy, h_prime_, sizeof(float) * maze_size);
        for (int y = 0; y < y_range_; y++) {
            for (int x = 0; x < x_range_; x++) {
                Coordinate point(x, y);
                if (!visited_[y * x_range_ + x])
                    continue;
                float h_min = FLT_MAX;
                for (Coordinate new_point : get_neighbors(point, g_)) {
                    int new_index = new_point.y * x_range_ + new_point.x;
                    float cost = calc_cost(point, new_point, g_);
                    float h_new = FLT_MAX;
                    if (!visited_[new_index]) {
                        h_new = cost + h_[new_index];
                    } else {
                        h_new = cost + h_prime_[new_index];
                    }
                    h_min = std::min(h_min, h_new);
                }
                h_prime_[y * x_range_ + x] = h_min;
            }
        }
        if (is_equal(h_prime_, h_prime_copy, maze_size))
            break;
    }

    for (int i = 0; i < maze_size; i++) {
        if (visited_[i]) {
            h_[i] = h_prime_[i];
        }
    }
    delete[] h_prime_copy;
}

bool LRTA_star::is_equal(float* m1, float* m2, int n) {
    for (int i = 0; i < n; i++) {
        if (m1[i] != m2[i]) {
            return false;
        }
    }
    return true;
}

void LRTA_star::extract_path(std::vector<Coordinate>& path,
                             const Coordinate start, const Coordinate* parent) {
    path.clear();
    // calculate path
    Coordinate cur_point = goal_;
    path.push_back(cur_point);
    while (cur_point != start) {
        Coordinate prev_point = parent[cur_point.y * x_range_ + cur_point.x];
        if (prev_point.x == -1 || prev_point.y == -1)
            break;
        cur_point = prev_point;
        path.push_back(cur_point);
    }
    std::vector<Coordinate> reversed_path;
    for (int i = path.size() - 1; i >= 0; i--) {
        reversed_path.push_back(path[i]);
    }
    path = reversed_path;
}

void LRTA_star::extract_path_in_close_set(std::vector<Coordinate>& path,
                                          Coordinate& cur_start) {
    path.clear();
    path.push_back(cur_start);
    Coordinate point = cur_start;

    while (true) {
        float h_min = FLT_MAX;
        Coordinate point_key;
        for (Coordinate new_point : get_neighbors(point, g_)) {
            int new_index = new_point.y * x_range_ + new_point.x;
            if (visited_[new_index]) {
                if (h_min > h_prime_[new_index]) {
                    h_min = h_prime_[new_index];
                    point_key = new_point;
                }
            } else {
                if (h_min > h_[new_index]) {
                    h_min = h_[new_index];
                    point_key = new_point;
                }
            }
        }
        path.push_back(point_key);
        point = point_key;
        if (!visited_[point_key.y * x_range_ + point_key.x]) {
            break;
        }
    }
    cur_start = point;
}

float LRTA_star::heuristic(const Coordinate& p) {
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

int main() {
    printf("Learning Real-Time A_star ... \n");
    Coordinate start = Coordinate(10, 5);
    Coordinate goal = Coordinate(45, 25);

    Maze maze = Maze();
    maze.init(start, goal);

    std::vector<std::vector<Coordinate>> visited_order_iter;
    std::vector<std::vector<Coordinate>> path_iter;

    LRTA_star searcher = LRTA_star();
    searcher.init(start, goal, 250, EUCLIDEAN);

    searcher.searching(path_iter, visited_order_iter, maze.g);
    maze.animation(path_iter, visited_order_iter);

    return 0;
}