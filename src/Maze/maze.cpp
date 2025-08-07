#include "maze.h"

#include <cstdio>

bool operator>(const CoordinatePair &p1, const CoordinatePair &p2) {
    return p1.first >= p2.first;
}

bool operator<(const CoordinatePair &p1, const CoordinatePair &p2) {
    return p1.first < p2.first;
}

Maze::Maze() {
    int maze_size = x_range_ * y_range_;
    g = new float[maze_size];
    for (int y = 0; y < y_range_; y++) {
        for (int x = 0; x < x_range_; x++) {
            g[y * x_range_ + x] = MAX_COST;
        }
    }
}

Maze::~Maze() { SAFE_DELETE(g); }

void Maze::init(Coordinate start, Coordinate goal) {
    start_ = start;
    goal_ = goal;

    img_width_ = x_range_ * grid_size_;
    img_height_ = y_range_ * grid_size_;
    maze_.create(img_height_, img_width_, CV_8UC3);
    maze_.setTo(cv::Scalar(255, 255, 255));

    init_maze();
    init_g();

    draw_scatter(start.x, start.y, RECTANGLE, cv::Scalar(255, 0, 0));
    draw_scatter(goal.x, goal.y, RECTANGLE, cv::Scalar(0, 255, 0));
}

void Maze::animation(std::vector<Coordinate> path,
                     std::vector<Coordinate> visited) {
    for (int i = 0; i < visited.size(); i++) {
        Coordinate point = visited[i];
        if (point == start_ || point == goal_)
            continue;
        draw_scatter(point.x, point.y, CIRCLE, cv::Scalar(128, 128, 128));
        cv::imshow("maze", maze_);
        cv::waitKey(1);
    }
    draw_path(path);
    cv::imshow("maze", maze_);
    cv::waitKey(0);
}

void Maze::animation(std::vector<Coordinate> path,
                     std::vector<Coordinate> visited_1,
                     std::vector<Coordinate> visited_2) {
    int size_1 = visited_1.size();
    int size_2 = visited_2.size();
    int max_size = std::max(size_1, size_2);
    int min_size = std::min(size_1, size_2);
    for (int i = 0; i < max_size; i++) {
        if (i < size_1) {
            Coordinate point_1 = visited_1[i];
            if (point_1 != start_ && point_1 != goal_)
                draw_scatter(point_1.x, point_1.y, CIRCLE,
                             cv::Scalar(128, 128, 128));
        }
        if (i < size_2) {
            Coordinate point_2 = visited_2[i];
            if (point_2 != start_ && point_2 != goal_)
                draw_scatter(point_2.x, point_2.y, CIRCLE,
                             cv::Scalar(225, 105, 65));
        }

        cv::imshow("maze", maze_);
        cv::waitKey(1);
    }
    draw_path(path);
    cv::imshow("maze", maze_);
    cv::waitKey(0);
}

void Maze::draw_path(std::vector<Coordinate> path) {
    for (int i = 0; i < path.size() - 1; i++) {
        Coordinate coord_1 = path[i];
        Coordinate coord_2 = path[i + 1];
        cv::Point2d center_1 = index_to_center(coord_1.x, coord_1.y);
        cv::Point2d center_2 = index_to_center(coord_2.x, coord_2.y);
        cv::line(maze_, center_1, center_2, cv::Scalar(0, 0, 255), 3);

        // cv::imshow("maze", maze_);
        // cv::waitKey(100);
    }
}

void Maze::draw_scatter(int x, int y, int type, cv::Scalar color) {
    cv::Point2d center = index_to_center(x, y);
    int half_grid_size = grid_size_ / 2;

    if (type == RECTANGLE) {
        cv::Point2d point_tl =
            cv::Point2d(center.x - half_grid_size, center.y - half_grid_size);
        cv::Point2d point_br =
            cv::Point2d(center.x + half_grid_size, center.y + half_grid_size);
        cv::rectangle(maze_, point_tl, point_br, color, cv::FILLED);
    } else if (type == CIRCLE) {
        cv::circle(maze_, center, half_grid_size, color, cv::FILLED);
    } else {
    }
}

cv::Point2d Maze::index_to_center(int x, int y) {
    int half_grid_size = grid_size_ / 2;
    cv::Point2d point;
    point.x = x * grid_size_ + half_grid_size;
    point.y = y * grid_size_ + half_grid_size;
    return point;
}

void Maze::init_maze() {
    for (int x = 0; x < x_range_; x++) {
        add_obstacle(x, 0);
        add_obstacle(x, y_range_ - 1);
    }

    for (int y = 0; y < y_range_; y++) {
        add_obstacle(0, y);
        add_obstacle(x_range_ - 1, y);
    }

    for (int x = 10; x < 21; x++) {
        add_obstacle(x, 15);
    }
    for (int y = 0; y < 15; y++) {
        add_obstacle(20, y);
    }

    for (int y = 15; y < 30; y++) {
        add_obstacle(30, y);
    }
    for (int y = 0; y < 16; y++) {
        add_obstacle(40, y);
    }
}

void Maze::init_g() {
    for (int x = 0; x < x_range_; x++) {
        g[x] = OBS_COST;
        g[(y_range_ - 1) * x_range_ + x] = OBS_COST;
    }

    for (int y = 0; y < y_range_; y++) {
        g[y * x_range_] = OBS_COST;
        g[y * x_range_ + (x_range_ - 1)] = OBS_COST;
    }

    for (int x = 10; x < 21; x++) {
        g[15 * x_range_ + x] = OBS_COST;
    }
    for (int y = 0; y < 15; y++) {
        g[y * x_range_ + 20] = OBS_COST;
    }

    for (int y = 15; y < 30; y++) {
        g[y * x_range_ + 30] = OBS_COST;
    }
    for (int y = 0; y < 16; y++) {
        g[y * x_range_ + 40] = OBS_COST;
    }
}

void Maze::add_obstacle(int x, int y) {
    cv::Point2d center = index_to_center(x, y);
    int half_grid_size = grid_size_ / 2;
    cv::Point2d point_tl =
        cv::Point2d(center.x - half_grid_size, center.y - half_grid_size);
    cv::Point2d point_br =
        cv::Point2d(center.x + half_grid_size, center.y + half_grid_size);
    cv::rectangle(maze_, point_tl, point_br, cv::Scalar(0, 0, 0), cv::FILLED);
}