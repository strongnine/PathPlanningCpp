#include <opencv2/opencv.hpp>
#include <vector>

#ifndef MAZE_H_
#define MAZE_H_

#define SAFE_DELETE(P)    \
    {                     \
        if (P)            \
            delete[] (P); \
        (P) = nullptr;    \
    }

#define OBS_COST (-(FLT_MAX / 2))
#define MAX_COST (FLT_MAX / 2)

enum ScatterTypes {
    CIRCLE = 1,
    RECTANGLE = 2,
};

struct Coordinate {
    Coordinate() : x(0), y(0), priority(0.f) {};
    Coordinate(int x, int y) : x(x), y(y), priority(0.f) {};
    Coordinate(int x, int y, float prior) : x(x), y(y), priority(prior) {};

    bool operator==(const Coordinate &p) const {
        return this->x == p.x && this->y == p.y;
    }

    bool operator!=(const Coordinate &p) const {
        return this->x != p.x || this->y != p.y;
    }

    bool operator>(const Coordinate &p) const {
        return this->x > p.x || (this->x == p.x && this->y > p.y);
    }

    bool operator<(const Coordinate &p) const {
        return this->x < p.x || (this->x == p.x && this->y < p.y);
    }

    int x;
    int y;
    float priority;
};

typedef std::pair<float, Coordinate> CoordinatePair;

bool operator>(const CoordinatePair &p1, const CoordinatePair &p2);
bool operator<(const CoordinatePair &p1, const CoordinatePair &p2);

struct CoordinateHash {
    std::size_t operator()(const Coordinate &p) const {
        std::size_t h1 = std::hash<int>()(p.x);
        std::size_t h2 = std::hash<int>()(p.y);
        return h1 ^ (h2 << 1);
    }
};

struct CoordinateEqual {
    bool operator()(const Coordinate &p1, const Coordinate &p2) const {
        return p1 == p2;
    }
};

class Maze {
 public:
    Maze();
    Maze(cv::Mat maze, int img_width, int img_height, Coordinate start,
         Coordinate goal)
        : maze_(std::move(maze)),
          img_width_(img_width),
          img_height_(img_height),
          start_(start),
          goal_(goal) {}
    ~Maze();

    void init(Coordinate start, Coordinate goal);
    void animation(std::vector<Coordinate> path,
                   std::vector<Coordinate> visited);
    void animation(std::vector<std::vector<Coordinate>> path_iter,
                   std::vector<std::vector<Coordinate>> visited_iter);
    void animation(std::vector<Coordinate> path,
                   std::vector<Coordinate> visited_1,
                   std::vector<Coordinate> visited_2);

    void set_png_save_path(std::string path);

    float *g;

 private:
    cv::Point2d index_to_center(int x, int y);
    void init_maze();
    void init_g();

    void draw_scatter(int x, int y, int type, cv::Scalar color);
    void draw_path(std::vector<Coordinate> path);
    void add_obstacle(int x, int y);

    void save_current_maze();

    cv::Mat maze_;
    int grid_size_ = 11;
    int x_range_ = 51;
    int y_range_ = 31;

    int img_width_, img_height_;

    Coordinate start_;
    Coordinate goal_;

    std::string png_save_path_ = "../data/png/";
    int frame_count_ = 0;
};

#endif